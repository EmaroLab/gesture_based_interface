#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
/**
 * @file
 */

/**@brief Class for estimating the Center of Mass 
 * 
 * Estimation of the position of the center of mass of filtered points in /camera/pcl_filtered via statistical mean.
 */
class PoseEstimation
{
public:
    /**
     * Transformation from Base Frame to Camera Frame
     */
    tf::TransformListener listener;
     
    /** Handler:
     * - subscribe to /camera/pcl_filtered to get filtered point cloud sent by pcl_filter
     * - publish odometry data on /odometry/baxter/center_of_mass
     * - publish a frame with origin in the computed center of mass and the same orientation of the Kinect 
     */
    PoseEstimation()
    {
        pcl_sub = nh.subscribe("/camera/pcl_filtered", 10, &PoseEstimation::poseCB, this);

        com_baxter_pub = nh.advertise<nav_msgs::Odometry>("/odometry/baxter/center_of_mass", 10);
    }
    /** 
     * Callback function
     * @param[in]  input	point cloud data from /camera/pcl_filtered
     */
    void poseCB(const sensor_msgs::PointCloud2 &input)
    {
		pcl::fromROSMsg(input, cloud);
		
		if(cloud.size() < 25)
			return;

		msg.header.stamp = ros::Time::now();
		x = 0, y = 0, z = 0;

		// Compute Center of Mass from statistical mean of all points
		for (size_t i = 0; i < cloud.points.size (); i++)
		{
			x += cloud.at(i).x;
			y += cloud.at(i).y;
			z += cloud.at(i).z;
		}
		x = x / (cloud.size() + 0.0);
		y = y / (cloud.size() + 0.0);
		z = z / (cloud.size() + 0.0);

		msg.header.frame_id =  "camera_depth_optical_frame";
		msg.child_frame_id =  "position_com_frame";

		msg.pose.pose.orientation.x = 0;              	// identity quaternion
		msg.pose.pose.orientation.y = 0;             	// identity quaternion
		msg.pose.pose.orientation.z = 0;            	// identity quaternion
		msg.pose.pose.orientation.w = 1;           		// identity quaternion
		msg.pose.covariance = {0.001, 0, 0, 0, 0, 0,  	// covariance on x
								0, 0.001, 0, 0, 0, 0,  	// covariance on y
								0, 0, 0.001, 0, 0, 0,  	// covariance on z
								0, 0, 0, 99999, 0, 0,  	// large covariance on rot x (not calculated)
								0, 0, 0, 0, 99999, 0,  	// large covariance on rot y (not calculated)
								0, 0, 0, 0, 0, 99999} ; // large covariance on rot z (not calculated)

		try{    
			listener.waitForTransform("world_frame", "camera_link", ros::Time(0), ros::Duration(10.0) );
			listener.lookupTransform("world_frame", "camera_link", ros::Time(0), t_world_to_camera);
			
			try{    
				listener.waitForTransform("camera_link", "camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0) );
				listener.lookupTransform("camera_link", "camera_depth_optical_frame",  ros::Time(0), t_camera_to_depth);
				tf::Vector3 point(x, y, z);
				tf::Vector3 point_bl =  t_world_to_camera * t_camera_to_depth * point;

				tf::vector3TFToMsg (point_bl, new_point);
				
				// Publish Odometry message with respect to the world frame
				msg.pose.pose.position.x = new_point.x;
				msg.pose.pose.position.y= new_point.y;
				msg.pose.pose.position.z= new_point.z;
				com_baxter_pub.publish(msg);

				// Transformation matrix of com_frame with respect to the camera
				tf::Transform transform;
				transform.setOrigin(tf::Vector3(x, y, z));  //origin in the center of mass
				transform.setRotation(tf::Quaternion(0, 0, 0, 1)); //same orientation

				// Publish the frame
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), msg.header.frame_id , msg.child_frame_id));
			}
			catch (tf::TransformException ex){
				ROS_WARN("Base to camera transform unavailable %s", ex.what());
			}
		}

		catch (tf::TransformException ex){

			ROS_WARN("Base to camera transform unavailable %s", ex.what());
		}		
}

protected:
	float x; /**< X coordinate of the Center Of Mass */
	float y; /**< Y coordinate of the Center Of Mass */
	float z; /**< Z coordinate of the Center Of Mass */	
	geometry_msgs::Vector3 new_point; /**< Point to publish */
	nav_msgs::Odometry msg;			  /**< Message to Publish */
	pcl::PointCloud<pcl::PointXYZ> cloud;	/**< Input cloud from /camera/pcl_filtered */
	tf::StampedTransform t_world_to_camera;	/**< Transformation from world frame to camera link frame*/
	tf::StampedTransform t_camera_to_depth;	/**< Transformation from camera link frame to camera depth optical frame*/
	tf::TransformBroadcaster br;			/**< Transformation Broadcaster for the center of mass frame */
    ros::NodeHandle nh;		/**< Node Handler */
    ros::Subscriber pcl_sub; /**< Subscriber to /camera/pcl_filtered */
    ros::Publisher com_baxter_pub; /**< Publisher of odometry data on /odometry/baxter/center_of_mass */
};
/** 
 * Main:
 * Initialization of the handler
 */
main(int argc, char **argv)
{
    ros::init(argc, argv, "position_estimation");

    ros::NodeHandle pnh;
    PoseEstimation handler;

    ros::spin();

    return 0;
}
