#include <ros/ros.h>
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
/**
 * @file
 */
 
using namespace std;

// Parameters
/** Distance to avoid security risks */
double security_distance = 1.5; 
/** Small threshold for checking attention */
double threshold = 0.7;

/** @brief Class to send the activate signal to the control board and the proximity alert
 * 
 * - check presence of a human in the area using beacons
 * - check attention, computing the rotation of the head with respect to the Kinect (yaw angle of the head)
 * - check security, computing the position of the head and the center of mass with respect to the world frame 
 */
class KinectActivate{
    /** KinectActivate:
     * - subscribe to /beacon/presence
     * - subscribe to /odometry/baxter/head
     * - subscribe to /odometry/baxter/center_of_mass
     */
    public:
    KinectActivate(){
        beacon_sub = nh.subscribe("/beacons/presence", 10, &KinectActivate::beaconCB, this);
        head_sub = nh.subscribe("/odometry/baxter/kinect_head", 10, &KinectActivate::headCB, this);
        beacon_sub = nh.subscribe("/odometry/baxter/center_of_mass", 10, &KinectActivate::comCB, this);
    }
    
    void reset(); /**< Metod to reset flags */
	bool isSecure(void); /**< Metod to check security */
	bool hasAttention(void); /**< Metod to check attention */
    /**
     * Presence callback function:
     *  sets a flag to true if beacons take over the presence of a human in the area 
     * @param[in]	input	presence of a human in the area (provided by beacon)
     */
    void beaconCB(const std_msgs::Float64 &input){
		// check presence
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
		// check attention
		tf::poseMsgToTF(head_odom.pose.pose, pose);
		yaw = fmod(tf::getYaw(pose.getRotation()), 2*M_PI);
		
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
		// check security
		if(com_odom.pose.pose.position.x < security_distance)
		    secure = false;
    }

protected:
	tf::Pose pose; /**< Current Odometry of the head */
	double yaw;	/**< Yaw angle of the head frame */
	
	// Flags
	bool beacon_presence = false; /**< Flag for identifying the presence of a human in the area, detected by beacons */
	bool attention = false;	/**< Flag for attention */
	bool secure = true;	/**< Flag for security */
	
    ros::NodeHandle nh;	/**< Private Node Handle */
    ros::Subscriber beacon_sub;  /**< Subscriber to /beacon/presence */
    ros::Subscriber head_sub;  /**< Subscriber to /odometry/kinect/head */
    ros::Subscriber com_sub;  /**< Subscriber to /odometry/kinect/center_of_mass */ 
};

/**
 *  Metod to reset flags
 */
void KinectActivate::reset(){ 
	beacon_presence = false; 
	attention = false; 
	secure = true; 
}
/**
 *  Metod to check security
 */
bool KinectActivate::isSecure(void){ 
	return beacon_presence && attention;
}
/**
 *  Metod to check attention
 */
bool KinectActivate::hasAttention(void){ 
	return secure;
}

/**
 * Main:
 * - initialization of kinectActivate
 * - publish activate signal on /presence
 * - publish security signal on /secure
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinect_activate");
	ros::NodeHandle nh("~");
	
	KinectActivate kinectActivate;
	
	nh.param<double>("security_distance", security_distance, 1.5);
	nh.param<double>("threshold", threshold, 0.7);
	
	ros::Rate r(10);
	
    ros::Publisher presence_pub;  
    ros::Publisher security_pub;   

    ros::NodeHandle n;
	presence_pub = n.advertise<std_msgs::Header>("/presence", 1);
	security_pub = n.advertise<std_msgs::Header>("/secure", 1);
	
	// Messages to publish
	std_msgs::Header presence_msg;
	std_msgs::Header security_msg;
	
	while(ros::ok())
	{	
		if(kinectActivate.hasAttention())
		{
			presence_msg.stamp = ros::Time::now();
			presence_pub.publish(presence_msg);
		}
		if(kinectActivate.isSecure()){
			security_msg.stamp = ros::Time::now();
			security_pub.publish(security_msg);
		}
		
		// Reset flags to false
		kinectActivate.reset();
		
		// Callbacks will reset flags to true if they match conditions
		ros::spinOnce();
		r.sleep();
	}
    return 0;
}
