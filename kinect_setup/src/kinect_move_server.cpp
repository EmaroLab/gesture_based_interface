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

/** 
 * Publisher, to control the tilt angle of the Kinect
*/
ros::Publisher rec_pub;

/** 
 * Callback of the /move_kinect service
 * @param[in]  req  Request Message
	* @param[in]  req.angle Desired Orientation angle of the Kinect
 * @param[out]  res    Response of the service
	* @param[out]  res.result If the operation is completed successfully
 */
bool move(kinect_setup::MoveKinect::Request  &req,
         kinect_setup::MoveKinect::Response &res)
{
	if(req.angle > 30)
	{
		req.angle = 30;
	}
	if(req.angle < -30)
	{
		req.angle = -30;
	}
	std_msgs::Float64 msg;
	msg.data = req.angle;
	rec_pub.publish(msg);
	res.result = true;
	return true;
}
/**
 * Main:
 * Initialization of the service
 */
main(int argc, char** argv)
{
    ros::init(argc, argv, "kinect_move_server");
	ros::NodeHandle n;
	rec_pub = n.advertise<std_msgs::Float64>("tilt_angle", 1000);
	ros::ServiceServer service = n.advertiseService("move_kinect", move);

    ros::spin();

    return 0;
}
