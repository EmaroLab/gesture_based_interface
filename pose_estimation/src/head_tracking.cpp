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

/**@brief Class for tracking the head 
 * 
 * The position of the head (info in /odometry/kinect/head) is followed by the Kinect. In particular, the Kinect changes its tilt angle and the ranges used by the filters are modified according to the xyz coordinates of the head.
 */
class HeadTracking
{
public:
     
    /** Handler:
     * - subscribe to /odometry/kinect/head to get odometry data
     * - use the service regulate_kinect_by_head to change the orientation of the Kinect and set filters ranges
     */
    HeadTracking()
    {
        head_sub = nh.subscribe("/odometry/kinect/head", 10, &HeadTracking::trackCB, this);
    }
    /** 
     * Callback function
     * @param[in]  head_pos	position of the head (xyz coordinates)
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
		ros::Subscriber head_sub; /**< Subscriber to /odometry/kinect/head */
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
