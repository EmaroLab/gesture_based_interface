#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <string> 
/**
 * @file
 */
 
ros::Publisher kinect_pub;  /**< Publisher to /odometry/kinect/<passed frame> */

ros::Publisher baxter_pub;   /**< Publisher to /odometry/baxter/<passed frame> */
    
ros::ServiceClient client_reset;  /**< Client to reset filters */
/**
 * Main function: 
 * 
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_tracker", ros::init_options::AnonymousName);
	ros::NodeHandle n("~");    
	static tf::TransformBroadcaster br;
	tf::TransformListener listener;
	
	std::string frame;
	n.param<std::string>("frame", frame, "kinect_head");
	
	std::stringstream kinect_path;
	kinect_path << "/odometry/kinect/" << frame;
	std::stringstream baxter_path;
	baxter_path << "/odometry/baxter/" << frame;
	
	ros::NodeHandle nh;  
	// Initilize Publishers for the odometry wrt Kinect
	kinect_pub = nh.advertise<nav_msgs::Odometry>(kinect_path.str(), 5);
	
	// Initilize Publishers for the odometry wrt Baxter
	baxter_pub = nh.advertise<nav_msgs::Odometry>(baxter_path.str(), 5);
	
    // Initialize Client to reset kinect
    client_reset = nh.serviceClient<std_srvs::Empty>("reset_kinect_filters");  
    
	tf::StampedTransform t_camera_to_frame;
	tf::StampedTransform t_world_to_camera;
	
	tf::Transform transform_kinect;
	tf::Transform transform_baxter;
	
	ros::Rate r(50);
	
	bool tracking = false;

	while(ros::ok()){
		try{    
			listener.waitForTransform("camera_link", frame, ros::Time::now(), ros::Duration(10.0) );
			listener.lookupTransform("camera_link", frame, ros::Time(0), t_camera_to_frame);
			
			// Update tranformation from camera_link frame (kinect) to frame
			transform_kinect = t_camera_to_frame;
			
			// Inizialize odometry message (wrt Kinect)
			nav_msgs::Odometry odom_msg;
			odom_msg.header.stamp = ros::Time::now();
			odom_msg.header.frame_id = "camera_link";
			odom_msg.child_frame_id = frame;
			tf::poseTFToMsg(transform_kinect, odom_msg.pose.pose);
			
			tracking = true;
			
			// publish message
			kinect_pub.publish(odom_msg);

			try{    
				listener.waitForTransform("world_frame", "camera_link", ros::Time::now(), ros::Duration(1.0) );
				listener.lookupTransform("world_frame", "camera_link", ros::Time(0), t_world_to_camera);
				// Update tranformation from world frame (Baxter) to frame
				transform_baxter = t_world_to_camera * transform_kinect;
				
				// Publish in broadcast the transformation
				br.sendTransform(tf::StampedTransform(transform_baxter, ros::Time::now(), "world_frame", frame));

				// Inizialize odometry message (wrt Baxter)
				odom_msg.header.stamp = ros::Time::now();
				odom_msg.header.frame_id = "world_frame";
				odom_msg.child_frame_id = frame;
				tf::poseTFToMsg(transform_baxter, odom_msg.pose.pose);

				// Publish message
				baxter_pub.publish(odom_msg);
			}
			catch (tf::TransformException &ex){
				ROS_WARN("Transform unavailable %s", ex.what());
			}
		}
		catch (tf::TransformException &ex){
			//ROS_WARN("Transform unavailable %s", ex.what());
			
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
