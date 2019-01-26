#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <math.h>
#include <boost/filesystem.hpp>
#include <Eigen/Geometry> 
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include "kinect_setup/MoveKinect.h"

double granularity = 1.0;

float angle = -30.0;

int go = 0;
int check = 0;

/** @brief Class to record environments
 * 
 * The class records all the environments for each possible orientation of the Kinect
 * 		- the range of the Kinect tilt is [-30,30] degree
 * 		- acquisitions with a given granularity
 */
class PclRecord{
	
	public:
	/** Handler:
	 * - subscribe to /camera/depth/points to get raw data from the Kinect
	 * - subscribe to /cur_tilt_angle
	 * - save environments in ~/.kinect_environments
	 */
	PclRecord(){
                pcl_sub = nh.subscribe("/camera/depth/points", 10, &PclRecord::PclRecordCB, this);
                angle_sub = nh.subscribe("/cur_tilt_angle", 10, &PclRecord::AngleCB, this);
	}
	/** 
	 * Callback function to save environments in a file environment<angle>.pcd in the folder /.kinect_environments
     * @param[in] cloud raw point cloud data from Kinect
	 */
	void PclRecordCB (const sensor_msgs::PointCloud2ConstPtr& cloud)
	{
		if ((cloud->width * cloud->height) == 0)
			return;
			
		if(go == 0)
			return;
		go = 0;
		
		const char *homedir;
		if ((homedir = getenv("HOME")) == NULL) {
			homedir = getpwuid(getuid())->pw_dir;
		}
		
		int print_angle = angle;
		std::stringstream ss2;
		ss2 << homedir << "/.kinect_environments/environment" << print_angle << ".pcd";
		
		ROS_INFO ("Data saved to %s", ss2.str ().c_str ());
		pcl::io::savePCDFile (ss2.str (), *cloud, Eigen::Vector4f::Zero (),
						 Eigen::Quaternionf::Identity (), false);
	}
	
	void AngleCB (const std_msgs::Float64 msg_angle)
	{
		if(abs(msg_angle.data - angle) < 0.5)
			check = 1;
	}

	protected:
		ros::NodeHandle nh;
        ros::Subscriber pcl_sub; /**< Subscriber to /camera/depth/points */
		ros::Subscriber angle_sub; /**< Subscriber to /cur_tilt_angle */

};

/**
 * Main:
 * Initialization of the parameter granularity and the handler.
 * Change the tilt angle of the Kinect in order to save all environments.
 * @param[in]  granularity  angle step between two consecutive records 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "recorder");

	system("mkdir --parents ~/.kinect_environments/");

	PclRecord handler;

	ros::NodeHandle n;
	n.param<double>("granularity", granularity, 1.0); //!!!

	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
		
	ros::ServiceClient client_move = n.serviceClient<kinect_setup::MoveKinect>("move_kinect");
	angle = -30.0;
	kinect_setup::MoveKinect srv;
	srv.request.angle = angle;
	client_move.call(srv);
	
	ros::WallDuration(1).sleep(); 
	
	srv.request.angle = angle;
	client_move.call(srv);
	ros::WallDuration(4).sleep(); 
		
	ROS_INFO ("Ready to start");
	while (ros::ok())
	{
		srv.request.angle = angle;
		client_move.call(srv);
		
		ROS_INFO ("Cur Angle %lf", angle);
		ros::WallDuration(2).sleep(); 
		
		go = 1;
		
		ros::WallDuration(2).sleep(); 
			
		angle = angle + granularity; //!!!
		
		if(angle >= 31.0)
			exit(0);
	}

	return 0;
}
