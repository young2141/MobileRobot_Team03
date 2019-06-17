#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <ros/callback_queue.h>
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

#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))

boost::mutex mutex[2];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;
bool detectedColor = false;
float pre_dAngleTurned;


struct pos{
	double _x;
	double _y;
};

template<typename T> inline bool isnan(T value)
{
	return value != value;
}

template<typename T> inline bool isinf(T value){
	return numeric_limits<T>::has_infinity() && 
		value == numeric_limits<T>::infinity();
}

void odomMsgCallback(const nav_msgs::Odometry &msg)
{
	// receive a '/odom' message with the mutex
	mutex[0].lock(); {
		g_odom = msg;
	} mutex[0].unlock();
}

void scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
	// receive a '/odom' message with the mutex
	mutex[1].lock(); {
		g_scan = msg;
	} 
	mutex[1].unlock();
}


tf::Transform getCurrentTransformation(void)
{
	tf::Transform transformation;

	nav_msgs::Odometry odom;

	mutex[0].lock(); {
		odom = g_odom;
	} 
	mutex[0].unlock();

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
		waitKey(30);
		return 1;
	}

    waitKey(30);
	return 0;
}

void sendMessage(const sensor_msgs::ImageConstPtr &msg){

	Mat img_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
	Scalar red(0, 64, 255); //BGR 순서
	Scalar green(0, 255, 0);
	Scalar yellow(0, 255, 255);
	//Scalar magenta(255, 0, 255);
   
            if(detectColor(green, img_frame) == 1) {
                cout << "<<green>>" << endl;
                /*geometry_msgs::Twist baseCmd;
                baseCmd.linear.x=0.2;
                baseCmd.linear.y=0.2;
                baseCmd.linear.z=0.2;
                pub.publish(baseCmd);

                baseCmd.angular.x = 0;
                baseCmd.angular.y = 0;
                baseCmd.angular.z = 0;
                pub.publish(baseCmd);
                sleep(1);*/
                return;
            }	
            else if(detectColor(yellow, img_frame) == 1) {
                cout << "\t<<yellow>>" << endl;
                return;
            }
            else if(detectColor(red, img_frame) == 1) {
                cout<<"\t\t<<red>>"<<endl;
                /*geometry_msgs::Twist baseCmd;
                baseCmd.linear.x=0;
                baseCmd.linear.y=0;
                baseCmd.linear.z=0;

                baseCmd.angular.x = 0;
                baseCmd.angular.y = 0;
                baseCmd.angular.z = 0;
                pub.publish(baseCmd);
                sleep(1);*/
                return;
            }
            else
            {
                detectedColor = false;
                cout<<"nothing going on "<<endl;
                cout<<detectedColor<<endl;
            }
            
    
}


bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
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
			pubTeleop.publish(baseCmd);
			loopRate.sleep();
		}
	}

	baseCmd.linear.x = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}

bool doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dTranslation, double dTranslationSpeed)
{
	geometry_msgs::Twist baseCmd;

	if(dTranslation < 0) {
		baseCmd.linear.x = -dTranslationSpeed;
	} else {
		baseCmd.linear.x = dTranslationSpeed;
	}

	baseCmd.linear.y = 0;
	baseCmd.angular.z = 0;

	bool bDone = false;
	ros::Rate loopRate(1000.0);

	while(ros::ok() && !bDone) {
		ros::spinOnce();

		tf::Transform currentTransformation = getCurrentTransformation();

		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
		double dDistMoved = relativeTransformation.getOrigin().length();

		if(fabs(dDistMoved) >= fabs(dTranslation)) {
			bDone = true;
			break;
		} else {
			pubTeleop.publish(baseCmd);

			loopRate.sleep();
		}
	}

	baseCmd.linear.x = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}

void convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<pos> &left_sides, vector<pos> &right_sides)
{
	int nRangeSize = (int)lrfScan.ranges.size();
	left_sides.clear();
	right_sides.clear();
	left_sides.resize(nRangeSize);
	right_sides.resize(nRangeSize);
	double toNan = 0.0;

	for(int i=0; i<nRangeSize; i++) {
		double dRange = lrfScan.ranges[i];
		pos _t;
		if(isnan(dRange)) {
			_t._x = 0.0 / toNan;
			_t._y = 0.0 / toNan;
			left_sides[i] = _t;
			right_sides[i] = _t;
		} 
		else {
			double dAngle = lrfScan.angle_min + i*lrfScan.angle_increment;
			//right side
			if(dAngle < 0.06){
				_t._x = dRange * cos(dAngle);
				_t._y = dRange * sin(dAngle);
				right_sides[i] = _t;
			}
			//left side
			else if(dAngle > 6.24){
				_t._x = dRange * cos(dAngle);
				_t._y = dRange * sin(dAngle);
				left_sides[i] = _t;
			}
			else{
				_t._x = 0.0 / toNan;
				_t._y = 0.0 / toNan;
				left_sides[i] = _t;
				right_sides[i] = _t;
			}
		}
	}
}

double calculate_average(vector<pos> &laserscanXY){
	double sum_ = 0;
	double laser_X,laser_Y;
	int valid_cnt = 0;

	for(int i=0;i<laserscanXY.size();i++){
		laser_X = laserscanXY[i]._x;
		laser_Y = laserscanXY[i]._y;		
		if(isnan(laser_X) || isnan(laser_Y)) 
			continue;
		sum_ += sqrt(pow(laser_X,2) + pow(laser_Y,2));
		valid_cnt++;
	}
	return sum_ / (double)valid_cnt;
}

int main(int argc, char **argv)
{
	int num;
	double dRotation_left, dRotation_right, dTranslation;
	double newx, newy, prevx, prevy;
	double left_average, right_average;
	
	ros::init(argc, argv, "team3_project_move");
	ros::NodeHandle nhp, nhs,nh,nhq;
	ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);
	ros::Subscriber subScan = nhs.subscribe("/scan", 10, &scanMsgCallback);
	ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);



    image_transport::ImageTransport it(nh);    
    image_transport::Subscriber subRGB = it.subscribe("/raspicam_node/image", 1, &sendMessage, ros::VoidPtr(), image_transport::TransportHints("compressed"));


	sensor_msgs::LaserScan scan;
	vector<pos> laserScanXY_left,laserScanXY_right;

	tf::Transform initialTransformation = getInitialTransformation();
	tf::Transform cur = initialTransformation;

	prevx = cur.getOrigin().getX();
	prevy = cur.getOrigin().getY();

	dRotation_left = -0.4;
	dRotation_right = 0.5;

	while(ros::ok()){
		cur = getCurrentTransformation();
		prevx = cur.getOrigin().getX();
		prevy = cur.getOrigin().getY();

		mutex[1].lock(); {
			scan = g_scan;
		} 
		mutex[1].unlock();
		convertScan2XYZs(scan, laserScanXY_left,laserScanXY_right);

		left_average = calculate_average(laserScanXY_left);
		right_average = calculate_average(laserScanXY_right);
		printf("\n-------------------------------------------------------------------\n");
		printf("x : %lf\ty : %lf\n", prevx, prevy);
		printf("장애물까지 거리 : %lf\n", (left_average + right_average) / 2.0);
        
            if(left_average < 0.5 || right_average < 0.5 ) {                
                printf("\n\t\t\t장애물발견\n\n");		
                doRotation(pub,cur,dRotation_left,0.2);
            }
            else{                
              //  doTranslation(pub,cur,0.1,0.2);
           }
            printf("-------------------------------------------------------------------\n");
	}


	return 0;
}
