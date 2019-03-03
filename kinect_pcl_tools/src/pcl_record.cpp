#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <math.h>
#include <Eigen/Geometry> 
#include <unistd.h>
#include <pwd.h>
#include "kinect_tracking_srvs/MoveKinect.h"
/**
 * @file
 */
 
/** Angle step between two consecutive records */	int granularity = 1;

/** Current tilt angle of the Kinect */			float angle = -30.0;
/** Flag for acquiring environments */			int go = 0;

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
		
		if ((homedir = getenv("HOME")) == NULL) {
			homedir = getpwuid(getuid())->pw_dir;
		}
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
		
		print_angle = (int)angle;
		save_path << homedir << "/.kinect_environments/environment" << print_angle << ".pcd";
		
		ROS_INFO ("Data saved to %s", save_path.str ().c_str ());
		pcl::io::savePCDFile (save_path.str (), *cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
		
		save_path.str(std::string());
	}

	protected:		
		std::stringstream save_path;  /**< Name of the file to store in the folder /.kinect_environments */
		const char *homedir; /**< Path from the home directory (value of the HOME environment variable) */
		int print_angle; /**< Angle of the Kinect */
		ros::NodeHandle nh; /**< Node Handle */
		ros::Subscriber pcl_sub; /**< Subscriber to /camera/depth/points */

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

	ros::NodeHandle nh("~");
	nh.param<int>("granularity", granularity, 1); 

	// Use 4 threads (to use WallDuration)
	ros::AsyncSpinner spinner(4); 
	spinner.start();
		
	ros::NodeHandle n;
	ros::ServiceClient client_move = n.serviceClient<kinect_tracking_srvs::MoveKinect>("move_kinect");
	
	// Move kinect to minimum angle
	angle = -30.0;
	kinect_tracking_srvs::MoveKinect srv;
	srv.request.angle = angle;
	client_move.call(srv);
	
	// Wait for complete
	ros::WallDuration(4).sleep(); 
	ros::spinOnce();
	
	while (ros::ok())
	{
		// Move to current angle
		srv.request.angle = angle;
		client_move.call(srv);
		ros::WallDuration(2).sleep(); 
		
		// Acquire background
		go = 1;
		
		ros::WallDuration(2).sleep(); 
			
		// Update angle
		angle = angle + granularity; 
		
		if(angle >= 31.0)
			exit(0);
	}

	return 0;
}
