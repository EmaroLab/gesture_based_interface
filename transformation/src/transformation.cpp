#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_broadcaster.h>

//transformation from Baxter to Kinect

/** 
 * Callback of the /move_kinect service
 * @param[in]  req  Request Message
	* @param[in]  req.angle Desired Orientation angle of the Kinect
 * @param[out]  res    Response of the service
	* @param[out]  res.result If the operation is completed successfully
 */
double x_Baxter;
double y_Baxter;
double z_Baxter;

double angle;

void angleCB(const std_msgs::Float64& angle_msg){
	if(angle_msg.data <= 30.0 && angle_msg.data >= -30.0)
	{
			angle = angle_msg.data;
	}
}
    
/**
 * Main
 */
main(int argc, char** argv)
{
    ros::init(argc, argv, "transformation");
	ros::NodeHandle n;
	
	n.param<double>("x_Baxter", x_Baxter, 0.3);
	n.param<double>("y_Baxter", y_Baxter, 0.3);
	n.param<double>("z_Baxter", z_Baxter, 0.3);
	
	ros::Subscriber angle_sub = nh.subscribe("/cur_tilt_angle", 10, angleCB, this);
	
	ros::Rate r(100);
	tf::TransformBroadcaster broadcaster;
	
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(x_Baxter, y_Baxter, z__Baxter));
    change_frame.setRotation(tf::Quaternion(0, 0, 0, 1)); //same orientation
    	
    tf::Transform change_frame2;
    change_frame2.setOrigin(tf::Vector3(0, 0, 0));
    change_frame2.setRotation(tf::Quaternion(0.5, 0.5, -0.5, 0.5)); //rotation matrix: [0 -1 0; 0 0 -1; 1 0 0];

        	
    tf::Transform change_frame3;
    change_frame3.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(0, angle, 0); //pitch = current tilt angle of the Kinect
    change_frame3.setRotation(frame_rotation);
    
	while(n.ok()){
		broadcaster.sendTransform(
			tf::StampedTransform(change_frame, ros::Time::now(),"baxter_frame","camera_link"));
			
		broadcaster.sendTransform(
			tf::StampedTransform(change_frame2, ros::Time::now(),"camera_depth_optical_frame","camera_link"));
		
		broadcaster.sendTransform(
			tf::StampedTransform(change_frame3, ros::Time::now(),"camera_depth_optical_frame","kinect_frame"));
				
		r.sleep();
	}

    return 0;
}
