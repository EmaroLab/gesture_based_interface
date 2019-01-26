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
#include <tf/transform_broadcaster.h>
#include "kinect_setup/RegulateKinectByHead.h"


ros::ServiceClient client_move;  

/**@brief Class for estimating the Center of Mass 
 * 
 * Follow of the position of the head from the info in /odometry/kinect/head
 */
class HeadTracking
{
public:
     
    /** Handler:
     * - subscribe to /camera/pcl_filtered to get filtered point cloud sent by plc_filter
     * - publish odometry data on /odometry/kinect/center_of_mass
     */
    HeadTracking()
    {
        head_sub = nh.subscribe("/odometry/kinect/head", 10, &HeadTracking::trackCB, this);
    }
    /** 
     * Callback function
     * @param[in]  input	point cloud data from /camera/pcl_filtered
     */
    void trackCB(const nav_msgs::Odometry &head_pos)
    {
		kinect_setup::RegulateKinectByHead srv;
		srv.request.x = -1 * head_pos.pose.pose.position.y;
		srv.request.y = -1 * head_pos.pose.pose.position.z;
		srv.request.z = head_pos.pose.pose.position.x;
		client_move.call(srv);
	
	}

	protected:
		ros::NodeHandle nh;
		ros::Subscriber head_sub; /**< Subscriber to /camera/pcl_filtered */
};
/**
 * Main:
 * Initialization of the handler
 */
main(int argc, char **argv)
{
    ros::init(argc, argv, "head_tracking");
    
    ros::NodeHandle pnh;
    client_move = pnh.serviceClient<kinect_setup::RegulateKinectByHead>("regulate_kinect_by_head");  

    HeadTracking handler;

    ros::spin();

    return 0;
}
