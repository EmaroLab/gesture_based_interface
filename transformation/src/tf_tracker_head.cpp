#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
/**
 * @file
 */
 
ros::Publisher head_kinect_pub;  /**< Publisher to /odometry/kinect/head */

ros::Publisher head_baxter_pub;   /**< Publisher to /odometry/baxter/head */
    
ros::ServiceClient client_reset;  /**< Client to reset filters */
/**
 * Main function: 
 * 
 */
main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_tracker_head");
	ros::NodeHandle n("~");    
	static tf::TransformBroadcaster br;
	tf::TransformListener listener;
	
	ros::NodeHandle nh;  
	// Initilize Publishers for the odometry wrt Kinect
	head_kinect_pub = nh.advertise<nav_msgs::Odometry>("/odometry/kinect/head", 10);
	
	// Initilize Publishers for the odometry wrt Baxter
	head_baxter_pub = nh.advertise<nav_msgs::Odometry>("/odometry/baxter/head", 10);
	
    // Initialize Client to reset kinect
    client_reset = nh.serviceClient<std_srvs::Empty>("reset_kinect_filters");  
    
	tf::StampedTransform t_camera_to_head;
	tf::StampedTransform t_world_to_camera;
	
	tf::Transform transform_kinect;
	tf::Transform transform_baxter;
	
	ros::Rate r(50);
	
	bool tracking = false;

	while(ros::ok()){
		try{    
			listener.waitForTransform("camera_link", "head", ros::Time::now(), ros::Duration(10.0) );
			listener.lookupTransform("camera_link", "head", ros::Time(0), t_camera_to_head);
			
			// Update tranformation from camera_link frame (kinect) to head frame
			transform_kinect = t_camera_to_head;
			
			// Inizialize odometry message (wrt Kinect)
			nav_msgs::Odometry odom_msg;
			odom_msg.header.stamp = ros::Time::now();
			odom_msg.header.frame_id = "camera_link";
			odom_msg.child_frame_id = "head";
			tf::poseTFToMsg(transform_kinect, odom_msg.pose.pose);
			
			tracking = true;
			
			// publish message
			head_kinect_pub.publish(odom_msg);

			try{    
				listener.waitForTransform("world_frame", "camera_link", ros::Time::now(), ros::Duration(1.0) );
				listener.lookupTransform("world_frame", "camera_link", ros::Time(0), t_world_to_camera);
				// Update tranformation from world frame (Baxter) to head frame
				transform_baxter = t_world_to_camera * transform_kinect;
				
				// Publish in broadcast the transformation
				br.sendTransform(tf::StampedTransform(transform_baxter, ros::Time::now(), "world_frame", "head"));

				// Inizialize odometry message (wrt Baxter)
				odom_msg.header.stamp = ros::Time::now();
				odom_msg.header.frame_id = "world_frame";
				odom_msg.child_frame_id = "head";
				tf::poseTFToMsg(transform_baxter, odom_msg.pose.pose);

				// Publish message
				head_baxter_pub.publish(odom_msg);
			}
			catch (tf::TransformException ex){
				ROS_WARN("Transform unavailable %s", ex.what());
			}
		}
		catch (tf::TransformException ex){
			ROS_WARN("Transform unavailable %s", ex.what());
			
			if(tracking){
				std_srvs::Empty srv;
				client_reset.call(srv);
				tracking = false;
			}
		}
		
		ros::spinOnce();
		r.sleep();
	}

    return 0;
}
