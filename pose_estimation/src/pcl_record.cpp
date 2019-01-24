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

float angle = -30.0;
double granularity = 1.0; //!!!
int go = 0;
/** @brief Class to record all the environments for each possible orientation of the Kinect
 * 		the range of the Kinect tilt is [-30,30] degree
 * 		with a given granularity
 */
class PclRecord{
	
	public:
	/** Handler:
	 * - subscribe to /camera/depth/points to get raw data from the Kinect
	 * - save environments in ~/.kinect_environments
	 */
        PclRecord(){
                pcl_sub = nh.subscribe("/camera/depth/points", 10, &cloudHandler::PclRecordCB, this);
	}
	/** 
	 * Callback function
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

	protected:
		ros::NodeHandle nh;
                ros::Subscriber pcl_sub;

};

/**
 * Main:
 * Initialization of the parameter granularity and the handler
 * @param[in]  granularity  angle step between two consecutive records 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "recorder");

	system("mkdir --parents ~/.kinect_environments/");

        PclRecord handler;

	ros::NodeHandle n;

	ros::Publisher rec_pub = n.advertise<std_msgs::Float64>("tilt_angle", 1000);

	ros::Rate loop_rate(0.3);
	ros::Rate sleep1(0.2);

	angle = -30.0;
	n.param<double>("granularity", granularity, 1.0); //!!!

	std_msgs::Float64 msg;
	msg.data = angle;
	rec_pub.publish(msg);
	ros::spinOnce();
	sleep1.sleep();
	ROS_INFO ("Ready to start");
	while (ros::ok())
	{
		msg.data = angle;
		rec_pub.publish(msg);
		go = 1;
		
		ROS_INFO ("Cur Angle %lf", angle);

		ros::spinOnce();
		loop_rate.sleep();
		
		angle = angle + granularity; //!!!
		
		if(angle >= 31.0)
			exit(0);
	}

	return 0;
}
