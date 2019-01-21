#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <math.h>
#include <string.h>
#include "std_msgs/Float64.h"
#include <Eigen/Geometry> 
#include <ros/package.h>
#include <pwd.h>

using namespace std;

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> backgrounds (61); 
pcl::PointCloud<pcl::PointXYZ>::Ptr actualImage(new pcl::PointCloud<pcl::PointXYZ>);

/**
 *  Class to implement Background Segmentation
 * 		remove the background associated to the current Kinect angle from the /camera/depth/points
 * 		publish filtered points cloud in /camera/pcl_background_segmentation
 */
double delta = 0.08;
int angle = 0;

class cloudHandler{
	/** Handler:
     * - subscribe to /camera/depth/points, raw data acquired by the Kinect
     * - subscribe to /cur_tilt_angle to acquire the current tilt angle of the Kinect
     * - publish to /camera/pcl_background_segmentation the filtered point cloud
     */
	public:
    	cloudHandler(){
        pcl_sub = nh.subscribe("/camera/depth/points", 10, &cloudHandler::cloudCB, this);
        angle_sub = nh.subscribe("/cur_tilt_angle", 10, &cloudHandler::angleCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/pcl_background_segmentation", 1);
    }
    /** 
     * Point cloud callback function
     */
    void cloudCB(const sensor_msgs::PointCloud2& input){  //or const sensor_msgs::PointCloud2ConstPtr& input
        
        sensor_msgs::PointCloud2 output;
        pcl::fromROSMsg(input, *actualImage);
        
        for (size_t i = 0; i < actualImage->points.size (); ++i){
      		if(abs(actualImage->points[i].z - backgrounds[angle + 30]->points[i].z) < delta){
        		actualImage->points[i].x = NAN;
        		actualImage->points[i].y = NAN;
        		actualImage->points[i].z = NAN;
      		}
  		}
  		 
        pcl::toROSMsg(*actualImage, output);
        pcl_pub.publish(output);
    }
    /** 
     * Angle callback function
     */
    void angleCB(const std_msgs::Float64& angle_msg){  
		if(angle_msg.data <= 30.0 && angle_msg.data >= -30.0)
		{
			angle = round(angle_msg.data);
		}
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Subscriber angle_sub;
    ros::Publisher pcl_pub;

};
/**
 * Main:
 * Initialization of the parameter delta and of the handler
 * @param[in]  delta	sensitivity of the segmentation
 */
main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_background_segmentation");
    ros::NodeHandle n("~");
    n.param<double>("delta", delta, 0.08);
    const char *homedir;
	if ((homedir = getenv("HOME")) == NULL) {
		homedir = getpwuid(getuid())->pw_dir;
	}
		
    std::string prefix = "/.kinect_environments/environment";
    std::string suffix = ".pcd";
    std::string name;
    for(int i = -30; i <= 30; i++)
    {
		pcl::PointCloud<pcl::PointXYZ>::Ptr curObj (new pcl::PointCloud<pcl::PointXYZ>);
		backgrounds[i + 30] = curObj;
		name = homedir + prefix + std::to_string(i) + suffix;
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (name, *(backgrounds[i + 30])) == -1) {
			PCL_ERROR ("Couldn't read file \n");
			return (-1);
		}
	}
	
	ROS_INFO ("Finish Loading Environments");
    cloudHandler handler;
    ros::spin();

    return 0;
}
