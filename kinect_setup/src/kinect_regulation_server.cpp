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
#include "kinect_setup/MoveKinect.h"
#include "kinect_setup/RegulateKinectByHead.h"
#include "kinect_setup/RegulateKinectByWrist.h"
#include "pose_estimation/SetFilterParam.h"
#include <math.h>

/** 
 * Publisher, to control the tilt angle of the Kinect
*/
ros::Publisher rec_pub;
/** 
 * Client, to control the tilt angle of the Kinect
*/
ros::ServiceClient client_move;
/** 
 * Client, to regulate the parameters of filters applied to the point cloud obtained by the background segmentation
*/
ros::ServiceClient client_filter;

/** Callback of the /regulate_kinect service
 * - control the tilt angle of the Kinect according to the coordinates xyz provided by the Beacon
 * - regulate the parameters of the filters according to the coordinates xyz
 * @param[in]  req  Request Message
	* @param[in]  req.x x-coordinate of the watch
	* @param[in]  req.y y-coordinate of the watch
	* @param[in]  req.z z-coordinate of the watch
 * @param[out]  res    Response of the service
	* @param[out]  res.result If the operation is completed successfully
 */
bool regulateWrist(kinect_setup::RegulateKinectByWrist::Request  &req,
         kinect_setup::RegulateKinectByWrist::Response &res)
{
	float x = req.x;
	float y = req.y;
	float z = req.z;
	
	float angle = -1 * atan(y/z) * 180 / M_PI;
	float min_z = z - 0.80;
	float max_z = z + 0.80;
	float min_y = y - 1.50;
	float max_y = y + 1.50;
	float min_x = x - 0.50;
	float max_x = x + 0.50;
	
	kinect_setup::MoveKinect srv;
	srv.request.angle = angle;
	
	client_move.call(srv);
	
	pose_estimation::SetFilterParam srv2;
	
	srv2.request.param_name = "min_z";
	srv2.request.value = min_z;
	client_filter.call(srv2);
	
	srv2.request.param_name = "max_z";
	srv2.request.value = max_z;
	client_filter.call(srv2);
	
	srv2.request.param_name = "min_y";
	srv2.request.value = min_y;
	client_filter.call(srv2);
	
	srv2.request.param_name = "max_y";
	srv2.request.value = max_y;
	client_filter.call(srv2);
	
	srv2.request.param_name = "min_x";
	srv2.request.value = min_x;
	client_filter.call(srv2);
	
	srv2.request.param_name = "max_x";
	srv2.request.value = max_x;
	client_filter.call(srv2);
	
	res.result = true;
	return true;
}

bool regulateHead(kinect_setup::RegulateKinectByHead::Request  &req,
         kinect_setup::RegulateKinectByHead::Response &res)
{
	float x = req.x;
	float y = req.y;
	float z = req.z;
	
	float angle = -1 * atan(y/z) * 180 / M_PI;
	
	float min_z = z - 0.80;
	float max_z = z + 0.80;
	float min_y = y - 0.30;
	float max_y = y + 1.90;
	float min_x = x - 0.50;
	float max_x = x + 0.50;
	
	kinect_setup::MoveKinect srv;
	srv.request.angle = angle;
	
	client_move.call(srv);
	
	pose_estimation::SetFilterParam srv2;
	
	srv2.request.param_name = "min_z";
	srv2.request.value = min_z;
	client_filter.call(srv2);
	
	srv2.request.param_name = "max_z";
	srv2.request.value = max_z;
	client_filter.call(srv2);
	
	srv2.request.param_name = "min_y";
	srv2.request.value = min_y;
	client_filter.call(srv2);
	
	srv2.request.param_name = "max_y";
	srv2.request.value = max_y;
	client_filter.call(srv2);
	
	srv2.request.param_name = "min_x";
	srv2.request.value = min_x;
	client_filter.call(srv2);
	
	srv2.request.param_name = "max_x";
	srv2.request.value = max_x;
	client_filter.call(srv2);
	
	res.result = true;
	return true;
}
/**
 * Main:
 * Initialization of the service
 */
main(int argc, char** argv)
{
    ros::init(argc, argv, "kinect_regulation_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("regulate_kinect_by_wrist", regulateWrist);
	ros::ServiceServer service2 = n.advertiseService("regulate_kinect_by_head", regulateHead);
	
	client_move = n.serviceClient<kinect_setup::MoveKinect>("move_kinect");
	client_filter = n.serviceClient<pose_estimation::SetFilterParam>("pcl_filter/set_filter_param");
	
    ros::spin();

    return 0;
}
