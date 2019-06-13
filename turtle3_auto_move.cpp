////////////////////////////////////
//aurthor : mobile robot_team03
//date : 2019.05.22
//name : auto move
////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <vector>
#include <limits>
#include <ctime>

using namespace std;

#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))

boost::mutex mutex[2];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;

float pre_dAngleTurned;

struct pos{
  double _x;
  double _y;
};

template<typename T>
inline bool isnan(T value)
{
    return value != value;
}

template<typename T>
inline bool isinf(T value){
    return std::numeric_limits<T>::has_infinity() && 
		value == std::numeric_limits<T>::infinity();
}

void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
    // receive a '/odom' message with the mutex
    mutex[0].lock(); {
        g_odom = msg;
    } mutex[0].unlock();
}

void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
    // receive a '/odom' message with the mutex
    mutex[1].lock(); {
        g_scan = msg;
    } mutex[1].unlock();
}


tf::Transform
getCurrentTransformation(void)
{
    tf::Transform transformation;

    nav_msgs::Odometry odom;

    mutex[0].lock(); {
        odom = g_odom;
    } mutex[0].unlock();

    transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

    transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

    return transformation;
}

tf::Transform
getInitialTransformation(void)
{
    tf::Transform transformation;

    ros::Rate loopRate(50.0);

    while(ros::ok()) {
        ros::spinOnce();

        transformation = getCurrentTransformation();

        if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
            break;
        } else {
            loopRate.sleep();
        }
    }

    return transformation;
}

bool
doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.linear.y = 0.0;

    if(dRotation < 0.) {
        baseCmd.angular.z = -dRotationSpeed;
    } else {
        baseCmd.angular.z = dRotationSpeed;
    }

    bool bDone = false;
    ros::Rate loopRate(50.0);



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
        } else {
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

bool
doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dTranslation, double dTranslationSpeed)
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
    ros::Rate loopRate(50.0);

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
void
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<pos> &left_sides, vector<pos> &right_sides)
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
        } else {
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
	if(isnan(laser_X) || isnan(laser_Y)) continue;
	sum_ += sqrt(pow(laser_X,2) + pow(laser_Y,2));
	valid_cnt++;
    }
    return sum_ / (double)valid_cnt;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle3_auto_move");

    ros::NodeHandle nhp, nhs;

    ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);
    ros::Subscriber subScan = nhs.subscribe("/scan", 10, &scanMsgCallback);
    ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    double dRotation_left;
    double dRotation_right;
    double dTranslation;
    srand((unsigned int)time(0));
    
    sensor_msgs::LaserScan scan;
    vector<pos> laserScanXY_left,laserScanXY_right;
    
    tf::Transform initialTransformation = getInitialTransformation();
    
    tf::Transform cur = initialTransformation;

    double nx,ny,cx,cy;
    double left_average_scan;
    double right_average_scan;

    cx = cur.getOrigin().getX();
    cy = cur.getOrigin().getY();

    dRotation_left = -0.4;
    dRotation_right = 0.5;

    while(ros::ok()){
	//ros::spinOnce();
	cur = getCurrentTransformation();

        cx = cur.getOrigin().getX();
        cy = cur.getOrigin().getY();
        //average scan 
	mutex[1].lock(); {
           scan = g_scan;
        } mutex[1].unlock();
	convertScan2XYZs(scan, laserScanXY_left,laserScanXY_right);
	//calculate average 
	left_average_scan = calculate_average(laserScanXY_left);
	right_average_scan = calculate_average(laserScanXY_right);
	std::cout << "\nlocation(x,y) : " << cx << " " << cy << std::endl;
	std::cout << "avg distance : " << (left_average_scan + right_average_scan) / 2.0 << std::endl;        
	// if(left_average_scan< 0.5 || right_average_scan < 0.5) {
	// 	std::cout << "장애물발견" << std::endl;       
	// 	doRotation(pub,cur,dRotation_left,0.4);
		
	// }
    //     else {
		doTranslation(pub, cur, 0.1, 0.4);
	//}
    }


    return 0;
}
