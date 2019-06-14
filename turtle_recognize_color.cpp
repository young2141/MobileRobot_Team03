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

using namespace cv;
using namespace std;
ros::Publisher pub;

void poseMessageReceivedRGB(const sensor_msgs::ImageConstPtr &msg)
{
    Mat img_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    Mat img_hsv;
    
    int range_count = 0;

	Scalar red(0, 0, 255);
	Scalar blue(255, 0, 0);
	Scalar yellow(0, 255, 255);
	Scalar magenta(255, 0, 255);


	Mat rgb_color = Mat(1, 1, CV_8UC3, red);
	Mat hsv_color;

	cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);

    int hue = (int)hsv_color.at<Vec3b>(0, 0)[0];
	int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
	int value = (int)hsv_color.at<Vec3b>(0, 0)[2];


	cout << "hue = " << hue << endl;
	cout << "saturation = " << saturation << endl;
	cout << "value = " << value << endl;

	int low_hue = hue - 4;
	int high_hue = hue + 4;
	int low_hue1 = 0, low_hue2 = 0;
	int high_hue1 = 0, high_hue2 = 0;

	if (low_hue < 10 ) {
		range_count = 2;

		high_hue1 = 180;
		low_hue1 = low_hue + 180;
		high_hue2 = high_hue;
		low_hue2 = 0;
	}

	else if (high_hue > 170) {
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


	cout << low_hue1 << "  " << high_hue1 << endl;
	cout << low_hue2 << "  " << high_hue2 << endl;

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
    if(left && top)
        if(width && height)
        {
            cout<<"<<red detected>>"<<endl;
                geometry_msgs::Twist baseCmd;
                baseCmd.linear.x=0;
                baseCmd.linear.y=0;
                baseCmd.linear.z=0;

                baseCmd.angular.x = 0;
                baseCmd.angular.y = 0;
                baseCmd.angular.z = 0;
           	pub.publish(baseCmd);
		sleep(1);
        }
            

	rectangle(img_frame, Point(left, top), Point(left + width, top + height),
			Scalar(0, 0, 255), 1);

    imshow("img_origin", img_mask1);
    imshow("squares", img_frame);
    waitKey(30);
}

int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "recognize_color");
    ros::NodeHandle nh,nhp;
;
   pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    image_transport::ImageTransport it(nh);
    // Create a subscriber object
    image_transport::Subscriber subRGB = it.subscribe("/raspicam_node/image", 1, &poseMessageReceivedRGB, ros::VoidPtr(), image_transport::TransportHints("compressed"));
    // Let ROS take over
    ros::spin();
    return 0;
	return 0;
}
