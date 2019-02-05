#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "kinect_setup/RegulateKinectByHead.h"
/**
 * @file
 */



/**@brief Class for tracking the head from the Kinect
 * 
 * The position of the head (info in /odometry/kinect/head) is followed by the Kinect. In particular, the Kinect changes its tilt angle and the ranges 
 * used by the filters are modified according to the xyz coordinates of the head.
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
   	client_move = nh.serviceClient<kinect_setup::RegulateKinectByHead>("regulate_kinect_by_head"); 
        head_sub = nh.subscribe("/odometry/kinect/kinect_head", 10, &HeadTracking::trackCB, this);
    }
    /** 
     * Callback function
     * @param[in]  head_pos	position of the head (xyz coordinates)
     */
    void trackCB(const nav_msgs::Odometry &head_pos)
    {
		kinect_setup::RegulateKinectByHead srv;
		// Rotation Matrix from camera_link to camera_depth_optical_frame 
		srv.request.x = -1 * head_pos.pose.pose.position.y;
		srv.request.y = -1 * head_pos.pose.pose.position.z;
		srv.request.z = head_pos.pose.pose.position.x;
		client_move.call(srv);
	}

	protected:
		ros::ServiceClient client_move; 
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

    HeadTracking handler;

    ros::spin();

    return 0;
}
