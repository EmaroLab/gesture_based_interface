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

using namespace std;
//Use this class to contain a public member that is used in the callback function

float delta = 0.08;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> backgrounds (61); 
pcl::PointCloud<pcl::PointXYZ>::Ptr actualImage(new pcl::PointCloud<pcl::PointXYZ>);

int angle = 0;


class cloudHandler{
	public:
    	cloudHandler(){
        pcl_sub = nh.subscribe("/camera/depth/points", 10, &cloudHandler::cloudCB, this);
        angle_sub = nh.subscribe("/cur_tilt_angle", 10, &cloudHandler::angleCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/pcl_background_segmentation", 1);
    }

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

main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_background_segmentation");
    
    std::string prefix = "/home/marco/catkin_ws/src/pose_estimation/src/environments/environment";
    std::string suffix = ".pcd";
    std::string name;
    for(int i = -30; i <= 30; i++)
    {
		pcl::PointCloud<pcl::PointXYZ>::Ptr curObj (new pcl::PointCloud<pcl::PointXYZ>);
		backgrounds[i + 30] = curObj;
		name = prefix + std::to_string(i) + suffix;
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
