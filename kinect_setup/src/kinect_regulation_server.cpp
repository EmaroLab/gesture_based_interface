#include <ros/ros.h>
#include "kinect_setup/MoveKinect.h"
#include "kinect_setup/RegulateKinectByHead.h"
#include "kinect_setup/RegulateKinectByWrist.h"
#include "pose_estimation/SetFilterParam.h"
#include "pose_estimation/SetFilter.h"
#include <math.h>
#include <std_srvs/Empty.h>

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
/** 
 * Client, to enable the filters applied to the point cloud obtained by the background segmentation
*/
ros::ServiceClient client_filter_enable;

int initial_angle = 0;


/** Callback of the regulate_kinect_by_wrist service
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
	float min_z = z - 0.50;
	float max_z = z + 0.50;
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

/** Callback of the regulate_kinect_by_head service
 * - control the tilt angle of the Kinect according to the coordinates xyz of the head
 * - regulate the parameters of the filters according to the coordinates xyz of the head
 * @param[in]  req  Request Message
	* @param[in]  req.x x-coordinate of the head
	* @param[in]  req.y y-coordinate of the head
	* @param[in]  req.z z-coordinate of the head
 * @param[out]  res    Response of the service
	* @param[out]  res.result If the operation is completed successfully
 */
bool regulateHead(kinect_setup::RegulateKinectByHead::Request  &req,
         kinect_setup::RegulateKinectByHead::Response &res)
{
	float x = req.x;
	float y = req.y;
	float z = req.z;
	
	float angle = -1 * atan(y/z) * 180 / M_PI;
	
	float min_z = z - 0.50;
	float max_z = z + 0.50;
	float min_y = y - 0.40;
	float max_y = y + 2.00;
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

/** Callback of the reset kinect filter service
 * - reset the tilt angle to initial angle
 * - reset the filters
 * @param[in]  request  Request Empty Message
 * @param[out]  response    Empty Response of the serviced
 */
bool resetKinect(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	float min_z = 0.01;
	float max_z = 4;
	float min_y = -1;
	float max_y = 1;
	float min_x = -1;
	float max_x = 1;
	
	// Reset angle of the kinect
	kinect_setup::MoveKinect srv_angle;
	srv_angle.request.angle = initial_angle;
	client_move.call(srv_angle);
	
	// Reset filter params
	pose_estimation::SetFilterParam srv;
	srv.request.param_name = "min_z";
	srv.request.value = min_z;
	client_filter.call(srv);
	
	srv.request.param_name = "max_z";
	srv.request.value = max_z;
	client_filter.call(srv);
	
	srv.request.param_name = "min_y";
	srv.request.value = min_y;
	client_filter.call(srv);
	
	srv.request.param_name = "max_y";
	srv.request.value = max_y;
	client_filter.call(srv);
	
	srv.request.param_name = "min_x";
	srv.request.value = min_x;
	client_filter.call(srv);
	
	srv.request.param_name = "max_x";
	srv.request.value = max_x;
	client_filter.call(srv);
	
	srv.request.param_name = "revert_x";
	srv.request.value = 1.0;
	client_filter.call(srv);
	
	srv.request.param_name = "revert_y";
	srv.request.value = 1.0;
	client_filter.call(srv);
	
	srv.request.param_name = "revert_z";
	srv.request.value = 1.0;
	client_filter.call(srv);
	
	// Reset all filters to true
	pose_estimation::SetFilter srv2;
	srv2.request.filter_name = "downsampling_filter";
	srv2.request.enable = true;
	client_filter_enable.call(srv2);
	
	srv2.request.filter_name = "x_filter";
	srv2.request.enable = true;
	client_filter_enable.call(srv2);
	
	srv2.request.filter_name = "y_filter";
	srv2.request.enable = true;
	client_filter_enable.call(srv2);
	
	srv2.request.filter_name = "z_filter";
	srv2.request.enable = true;
	client_filter_enable.call(srv2);
	
	srv2.request.filter_name = "sor_filter";
	srv2.request.enable = true;
	client_filter_enable.call(srv2);

	return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinect_regulation_server");
	ros::NodeHandle n;
	
	// Advertise Services
	ros::ServiceServer service = n.advertiseService("regulate_kinect_by_wrist", regulateWrist);
	ros::ServiceServer service2 = n.advertiseService("regulate_kinect_by_head", regulateHead);
	ros::ServiceServer service3 = n.advertiseService("reset_kinect_filters", resetKinect);
	
	// Initialize clients
	client_move = n.serviceClient<kinect_setup::MoveKinect>("move_kinect");
	client_filter = n.serviceClient<pose_estimation::SetFilterParam>("pcl_filter/set_filter_param");
	client_filter_enable = n.serviceClient<pose_estimation::SetFilter>("pcl_filter/set_filter");
	
	// Acquire param
	ros::NodeHandle nh("~");
    n.param<int>("initial_angle", initial_angle, 0);
    
    ros::spin();

    return 0;
}
