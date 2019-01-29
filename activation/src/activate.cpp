#include <ros/ros.h>
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <math.h>

using namespace std;

double distance_head = 0.0;
double distance_com = 0.0;

bool beacon_presence = false;
bool attention = false;
bool secure = false;

double security_distance = 1.5;
double threshold = 0.7;


/** @brief Class to send the activate signal to the control board and the proximity alert
 * 
 * - check presense of a human in the area using beacons
 * - check attention computing the rotation of the head with respect to the Kinect (yaw angle of the head)
 * - check security computing the position of the head with respect to the world frame
 */
class KinectActivate{
    /** KinectActivate:
     * - subscribe to /beacon/presence
     * - subscribe to /odometry/kinect/head
     * - subscribe to /odometry/kinect/center_of_mass
     */
    public:
    KinectActivate(){
        beacon_sub = nh.subscribe("/beacon/presence", 10, &KinectActivate::beaconCB, this);
        head_sub = nh.subscribe("/odometry/kinect/head", 10, &KinectActivate::headCB, this);
        beacon_sub = nh.subscribe("/odometry/kinect/center_of_mass", 10, &KinectActivate::comCB, this);
    }
    /**
     * Presence callback function:
     *  sets a flag to true if beacon takes over the presence of a human in the area 
     * @param[in]	input	presense of a human in the area (provided by beacon)
     */
    void beaconCB(const std_msgs::Float64 &input){
		// check presense
		if(input.data == 1.0){
			beacon_presence = true;
		}
    }
    /**
     * Callback function using odometry messages of the head:
     * - check attention computing the rotation of the head with respect to the Kinect (yaw angle of the head)
     * - check security computing the position of the head with respect to the world frame
     * @param[in]	head_odom	odometry messages of the head
     */
    void headCB(const nav_msgs::Odometry &head_odom){
		double roll, pitch, yaw;
		
		// check attention
		tf::Pose pose;
		tf::poseMsgToTF(head_odom.pose.pose, pose);
		yaw = fmod(tf::getYaw(pose.getRotation()), 2*M_PI);
		//ROS_INFO("%lf", yaw);
		if(abs(yaw - M_PI/2) < threshold){
			ROS_INFO("Attention");
			attention = true;
		}
		// check security
		if(head_odom.pose.pose.position.x < security_distance)
		    secure = false;
			
    }
    /**
     * Callback function using odometry messages of the center of mass:
     * - check security computing the position of the center of mass with respect to the world frame 
     * @param[in]	com_odom	odometry messages of the center of mass
     */
    void comCB(const nav_msgs::Odometry &com_odom){
		if(com_odom.pose.pose.position.x < security_distance)
		    secure = false;
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber beacon_sub;  /**< Subscriber to /beacon/presence */
    ros::Subscriber head_sub;  /**< Subscriber to /odometry/kinect/head */
    ros::Subscriber com_sub;  /**< Subscriber to /odometry/kinect/center_of_mass */ 
};

/**
 * Main:
 * - initialization of kinectActivate
 * - publish activate signal on /presence
 * - publish security signal on /secure
 */
main(int argc, char** argv)
{
    ros::init(argc, argv, "kinect_activate");
	ros::NodeHandle nh;
	
	KinectActivate kinectActivate;
	
	nh.param<double>("security_distance", security_distance, 1.5);
	nh.param<double>("threshold", threshold, 0.7);
	
	ros::Rate r(1000);
	
    ros::Publisher presence_pub;  /**< Publisher of presence signal on /presence */
    ros::Publisher security_pub;  /**< Publisher of security signal on /secure */ 

	presence_pub = nh.advertise<std_msgs::Header>("/presence", 1);
	security_pub = nh.advertise<std_msgs::Header>("/secure", 1);
	
	ros::Time now;
	// Header msgs
	std_msgs::Header presence_msg;
	std_msgs::Header security_msg;
	while(ros::ok())
	{	
		if(beacon_presence && attention)
		{
			presence_msg.stamp = ros::Time::now();
			presence_pub.publish(presence_msg);
		}
		if(secure){
			security_msg.stamp = ros::Time::now();
			security_pub.publish(security_msg);
		}
		beacon_presence = false;
		attention = false;
		secure = true;
		
		ros::spinOnce();
		r.sleep();
	}
    return 0;
}
