#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <vector>
#include <limits>
#include <ctime>

using namespace std;
using namespace cv;

ros::Publisher pub;
boost::mutex mutex;
nav_msgs::Odometry g_odom;
float pre_dAngleTurned;

Scalar red(0, 64, 255); //BGR 순서
Scalar green(0, 255, 0);
Scalar yellow(0, 255, 255);
Scalar pink(204,102,255);
Scalar orange(0,140,255);
Scalar blue(255,0,0);

void odomMsgCallback(const nav_msgs::Odometry &msg)
{
    mutex.lock(); {
        g_odom = msg;
    } mutex.unlock();
}

tf::Transform getCurrentTransformation(void)
{
    tf::Transform transformation;
    nav_msgs::Odometry odom;

    mutex.lock(); {
        odom = g_odom;
    } 
    mutex.unlock();

    transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
    transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

    return transformation;
}

tf::Transform getInitialTransformation(void)
{
    tf::Transform transformation;
    ros::Rate loopRate(1000.0);

    while(ros::ok()) {
        ros::spinOnce();
        transformation = getCurrentTransformation();

        if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
            break;
        } 
	else {
            loopRate.sleep();
        }
    }
    return transformation;
}

bool doRotation(tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
	geometry_msgs::Twist baseCmd;
	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;

	if(dRotation < 0.) {
		baseCmd.angular.z = -dRotationSpeed;
	}
	else {
		baseCmd.angular.z = dRotationSpeed;
	}

	bool bDone = false;
	ros::Rate loopRate(1000.0);

	while(ros::ok() && !bDone) {
		ros::spinOnce();
		tf::Transform currentTransformation = getCurrentTransformation();

		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
		tf::Quaternion rotationQuat = relativeTransformation.getRotation();

		double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));

		if( fabs(dAngleTurned) > fabs(dRotation) || (dRotation == 0)) 
		{
			bDone = true;
			break;
		} 
		else {
			pre_dAngleTurned = dAngleTurned;
			pub.publish(baseCmd);
			loopRate.sleep();
		}
	}

	baseCmd.linear.x = 0.0;
	baseCmd.angular.z = 0.0;
	pub.publish(baseCmd);

	return bDone;
}

int detectColor(Scalar want2detect, Mat img_frame)
{
	Mat img_hsv;
    
	int range_count = 0;
	Mat rgb_color = Mat(1, 1, CV_8UC3, want2detect); // <<-- 여기 색상을 바꿔줘야 함.
 	Mat hsv_color;
	cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);

	int hue = (int)hsv_color.at<Vec3b>(0, 0)[0];
	int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
	int value = (int)hsv_color.at<Vec3b>(0, 0)[2];
	int low_hue = hue - 4;
	int high_hue = hue + 4;
	int low_hue1 = 0, low_hue2 = 0;
	int high_hue1 = 0, high_hue2 = 0;

	if (low_hue < 10 ) {
		range_count = 2;

		high_hue1 = 180;
		low_hue1 = low_hue + 170;
		high_hue2 = high_hue;
		low_hue2 = 0;
	}

	else if (high_hue > 180) {
		range_count = 2;

		high_hue1 = low_hue;
		low_hue1 = 180;
		high_hue2 = high_hue - 180;
		low_hue2 = 0;
	}

	else {
		range_count = 1;

		low_hue1 = low_hue;
		high_hue1 = high_hue;
	}

	//HSV로 변환
	cvtColor(img_frame, img_hsv, COLOR_BGR2HSV);

	//지정한 HSV 범위를 이용하여 영상을 이진화
	Mat img_mask1, img_mask2;
	inRange(img_hsv, Scalar(low_hue1, 50, 50), Scalar(high_hue1, 255, 255), img_mask1);
	if (range_count == 2) {
		inRange(img_hsv, Scalar(low_hue2, 50, 50), Scalar(high_hue2, 255, 255), img_mask2);
		img_mask1 |= img_mask2;
	}

	//morphological opening 작은 점들을 제거 
	erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing 영역의 구멍 메우기 
	dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//라벨링 
	Mat img_labels, stats, centroids;
	int numOfLables = connectedComponentsWithStats(img_mask1, img_labels,
			stats, centroids, 8, CV_32S);

	//영역박스 그리기
	int max = -1, idx = 0;
	for (int j = 1; j < numOfLables; j++) {
		int area = stats.at<int>(j, CC_STAT_AREA);
		if (max < area)
		{
			max = area;
			idx = j;
		}
	}

	int left = stats.at<int>(idx, CC_STAT_LEFT);
	int top = stats.at<int>(idx, CC_STAT_TOP);
	int width = stats.at<int>(idx, CC_STAT_WIDTH);
	int height = stats.at<int>(idx, CC_STAT_HEIGHT);

	if(left && top && width && height) {
		rectangle(img_frame, Point(left, top), Point(left + width, top + height), Scalar(0, 0, 255), 1);
		imshow("squares", img_frame);
		return 1;
	}
	waitKey(30);
	return 0;
}

void sendVel(float x1, float y1, float z1, float x2, float y2, float z2)
{
	geometry_msgs::Twist baseCmd;
	baseCmd.linear.x = x1;
 	baseCmd.linear.y = y1;
 	baseCmd.linear.z = z1;

	baseCmd.angular.x = x2;
	baseCmd.angular.y = y2;
	baseCmd.angular.z = z2;
	pub.publish(baseCmd);
}

void sendMessage(const sensor_msgs::ImageConstPtr &msg){
	Mat img_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
 	
	double currentx, currenty;
	tf::Transform current;
	current = getCurrentTransformation();
	currentx = current.getOrigin().getX();
	currenty = current.getOrigin().getY();
	
	if(detectColor(green, img_frame) == 1) {
		//go straight
		cout << "<<green>>" << endl;
        sendVel(0.07, 0, 0, 0, 0, 0);
	}	
	else if(detectColor(yellow, img_frame) == 1) {
		//turn left
		cout << "\t<<yellow>>" << endl;
		doRotation(current, -0.3, 0.5);
		sendVel(0.07, 0, 0, 0, 0, 0);
	}
	else if(detectColor(blue, img_frame) == 1) {
		//turn right
		cout << "\t\t<<blue>>" << endl;
		doRotation(current, 0.3, 0.5);
		sendVel(0.07, 0, 0, 0, 0, 0);
	}
	else if(detectColor(pink, img_frame) == 1){
		//go slowly
		cout<<"\t\t<<pink"<<endl;
		sendVel(0.01,0,0,0,0,0);
	}
	else if(detectColor(red, img_frame) == 1) {
		//stop
		cout<<"\t\t<<red>>"<<endl;
		sendVel(0, 0, 0, 0, 0, 0);
	}
	 else {
	 	imshow("squares", img_frame);
	 	cout << "not detected" << endl;
	}
}

int main(int argc, char **argv)
{
    // Initialize the ROS system
 	ros::init(argc, argv, "team3_project_color");
	ros::NodeHandle nh, nhs, nhp;

	ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber subRGB = it.subscribe("/raspicam_node/image", 1, &sendMessage, ros::VoidPtr(), image_transport::TransportHints("compressed"));
	pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	geometry_msgs::Twist baseCmd;

	/*baseCmd.linear.x=0.03;
    baseCmd.linear.y=0;
    baseCmd.linear.z=0;
	baseCmd.angular.x = 0;
    baseCmd.angular.y = 0;
    baseCmd.angular.z = 0;
    pub.publish(baseCmd);*/

    // Create a subscriber object
    // Let ROS take over
	ros::spin();
	return 0;
}
