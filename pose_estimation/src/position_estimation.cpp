#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/common/centroid.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>





class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("/camera/pcl_filtered", 10, &cloudHandler::cloudCB, this);


        pcl_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    }
    
    tf::TransformListener listener;

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        
        pcl::fromROSMsg(input, cloud);

        nav_msgs::Odometry msg;

        
        msg.header.stamp = ros::Time::now();

        geometry_msgs::Vector3 new_point;


        float x = 0, y = 0, z = 0;

        for (size_t i = 0; i < cloud.points.size (); i++)
        {
            x += cloud.at(i).x;
            y += cloud.at(i).y;
            z += cloud.at(i).z;
        }
        x = x / (cloud.size() + 0.0);
        y = y / (cloud.size() + 0.0);
        z = z / (cloud.size() + 0.0);


        msg.header.frame_id =  "camera_link";
        msg.child_frame_id =  "position_human_frame";
        
        msg.pose.pose.orientation.x = 1 ;              // identity quaternion
        msg.pose.pose.orientation.y = 0  ;             // identity quaternion
        msg.pose.pose.orientation.z = 0   ;            // identity quaternion
        msg.pose.pose.orientation.w = 0    ;           // identity quaternion
        msg.pose.covariance = {0.001, 0, 0, 0, 0, 0,  // covariance on gps_x
								0, 0.001, 0, 0, 0, 0,  // covariance on gps_y
								0, 0, 0.001, 0, 0, 0,  // covariance on gps_z
								0, 0, 0, 99999, 0, 0,  // large covariance on rot x
								0, 0, 0, 0, 99999, 0,  // large covariance on rot y
								0, 0, 0, 0, 0, 99999} ; // large covariance on rot z

   
  
        
	tf::Vector3 point(x, y, z);

		tf::StampedTransform transformation;
	try{    
		listener.lookupTransform( "camera_link","camera_link",  
		ros::Time::now(), transformation);
	}

	catch (tf::TransformException ex){

		ROS_WARN("Base to camera transform unavailable %s", ex.what());
	}

	tf::Vector3 point_bl =  transformation * point;

	tf::vector3TFToMsg (point_bl, new_point);

	msg.pose.pose.position.x = new_point.x ;            // x measurement GPS.
	msg.pose.pose.position.y= new_point.y;             // y measurement GPS.
	msg.pose.pose.position.z= new_point.z;




	pcl_pub.publish(msg);

    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "position_estimation");

    cloudHandler handler;

    ros::spin();

    return 0;
}
