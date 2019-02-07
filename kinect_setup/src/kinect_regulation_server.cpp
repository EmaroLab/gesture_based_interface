#include <ros/ros.h>
#include "kinect_tracking_srvs/MoveKinect.h"
#include "kinect_tracking_srvs/RegulateKinectByHead.h"
#include "kinect_tracking_srvs/RegulateKinectByWrist.h"
#include "kinect_filter_srvs/SetFilterParam.h"
#include "kinect_filter_srvs/SetFilter.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <string> 

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

//
void set_filter_param(std::string param, float value)
{
	kinect_filter_srvs::SetFilterParam srv;
    srv.request.param_name = param;
    srv.request.value = value;
    client_filter.call(srv); 
}

void set_filter(std::string param, bool enable)
{
	kinect_filter_srvs::SetFilter srv;
    srv.request.filter_name = param;
    srv.request.enable = enable;
    client_filter_enable.call(srv); 
}

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
bool regulateWrist(kinect_tracking_srvs::RegulateKinectByWrist::Request  &req,
         kinect_tracking_srvs::RegulateKinectByWrist::Response &res)
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
	
	kinect_tracking_srvs::MoveKinect srv_angle;
	srv_angle.request.angle = angle;
	
	client_move.call(srv_angle);
	
	set_filter_param("min_z", min_z);
	set_filter_param("max_z", max_z);
	set_filter_param("min_y", min_y);
	set_filter_param("max_y", max_y);
	set_filter_param("min_x", min_x);
	set_filter_param("max_x", max_x);

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
bool regulateHead(kinect_tracking_srvs::RegulateKinectByHead::Request  &req,
         kinect_tracking_srvs::RegulateKinectByHead::Response &res)
{
	float x = req.x;
	float y = req.y;
	float z = req.z;
	
	float angle = -1 * atan(y/z) * 180 / M_PI;
	
	float min_z = z - 0.50;
	float max_z = z + 0.15;
	float min_y = y - 0.40;
	float max_y = y + 1.50;
	float min_x = x - 0.50;
	float max_x = x + 0.50;
	
	kinect_tracking_srvs::MoveKinect srv_angle;
	srv_angle.request.angle = angle;
	
	client_move.call(srv_angle);
	
	
	set_filter_param("min_z", min_z);
	set_filter_param("max_z", max_z);
	set_filter_param("min_y", min_y);
	set_filter_param("max_y", max_y);
	set_filter_param("min_x", min_x);
	set_filter_param("max_x", max_x);

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
	kinect_tracking_srvs::MoveKinect srv_angle;
	srv_angle.request.angle = initial_angle;
	client_move.call(srv_angle);
	
	// Reset filter params	
	set_filter_param("min_z", min_z);
	set_filter_param("max_z", max_z);
	set_filter_param("min_y", min_y);
	set_filter_param("max_y", max_y);
	set_filter_param("min_x", min_x);
	set_filter_param("max_x", max_x);
	
	set_filter_param("revert_x", 1.0);
	set_filter_param("revert_y", 1.0);
	set_filter_param("revert_z", 1.0);
	
	// Reset all filters to true	
	set_filter("downsampling_filter", true);
	set_filter("x_filter", true);
	set_filter("y_filter", true);
	set_filter("z_filter", true);
	set_filter("sor_filter", true);

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
	client_move = n.serviceClient<kinect_tracking_srvs::MoveKinect>("move_kinect");
	client_filter = n.serviceClient<kinect_filter_srvs::SetFilterParam>("pcl_filter/set_filter_param");
	client_filter_enable = n.serviceClient<kinect_filter_srvs::SetFilter>("pcl_filter/set_filter");
	
	// Acquire param
	ros::NodeHandle nh("~");
    n.param<int>("initial_angle", initial_angle, 0);
    
    ros::spin();

    return 0;
}
