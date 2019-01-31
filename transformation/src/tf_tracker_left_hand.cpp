#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
/**
 * @file
 */
 
ros::Publisher left_hand_kinect_pub;  /**< Publisher to /odometry/kinect/left_hand */

ros::Publisher left_hand_baxter_pub;   /**< Publisher to /odometry/baxter/left_hand */
    
/**
 * Main function: 
 * 
 */
main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_tracker_left_hand");
	ros::NodeHandle n("~");    
	static tf::TransformBroadcaster br;
	tf::TransformListener listener;
	
	
	ros::NodeHandle nh;  
	// Initilize Publishers for the odometry wrt Kinect
	left_hand_kinect_pub = nh.advertise<nav_msgs::Odometry>("/odometry/kinect/left_hand", 10);
	
	// Initilize Publishers for the odometry wrt Baxter
	left_hand_baxter_pub = nh.advertise<nav_msgs::Odometry>("/odometry/baxter/left_hand", 10);
	
	tf::StampedTransform t_camera_to_left_hand;
	tf::StampedTransform t_world_to_camera;
	
	tf::Transform transform_kinect;
	tf::Transform transform_baxter;
	
	
	ros::Rate r(10000);
	while(ros::ok()){
		try{    
			listener.waitForTransform("camera_link", "left_hand", ros::Time::now(), ros::Duration(3.0) );
			listener.lookupTransform("camera_link", "left_hand", ros::Time(0), t_camera_to_left_hand);
			
			transform_kinect = t_camera_to_left_hand;
			
			// Inizialize odometry message (wrt Kinect)
			nav_msgs::Odometry odom_msg;
			odom_msg.header.stamp = ros::Time::now();
			odom_msg.header.frame_id = "camera_link";
			odom_msg.child_frame_id = "left_hand";
			tf::poseTFToMsg(transform_kinect, odom_msg.pose.pose);
			
			// publish message
			left_hand_kinect_pub.publish(odom_msg);

			try{    
				listener.waitForTransform("world_frame", "camera_link", ros::Time::now(), ros::Duration(1.0) );
				listener.lookupTransform("world_frame", "camera_link", ros::Time(0), t_world_to_camera);
				// Update tranformation from world frame (Baxter) to head frame
				transform_baxter = t_world_to_camera * transform_kinect;
				
				// Publish in broadcast the transformation
				br.sendTransform(tf::StampedTransform(transform_baxter, ros::Time::now(), "world_frame", "left_hand"));

				// Inizialize odometry message (wrt Baxter)
				odom_msg.header.stamp = ros::Time::now();
				odom_msg.header.frame_id = "world_frame";
				odom_msg.child_frame_id = "left_hand";
				tf::poseTFToMsg(transform_baxter, odom_msg.pose.pose);

				// Publish message
				left_hand_baxter_pub.publish(odom_msg);
			}
			catch (tf::TransformException ex){
				ROS_WARN("Transform unavailable %s", ex.what());
			}
		}
		catch (tf::TransformException ex){
			ROS_WARN("Transform unavailable %s", ex.what());
		}
		
		ros::spinOnce();
		r.sleep();
	}

    return 0;
}
