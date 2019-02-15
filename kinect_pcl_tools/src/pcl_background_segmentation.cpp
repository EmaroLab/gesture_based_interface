#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <cstring>
#include "std_msgs/Float64.h"
#include <Eigen/Geometry>
#include <ros/package.h>
#include <pwd.h>
/**
 * @file
 */
 
using namespace std;

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> backgrounds (61);
pcl::PointCloud<pcl::PointXYZ>::Ptr actualImage(new pcl::PointCloud<pcl::PointXYZ>);

double delta = 0.08;
int granularity = 1;
int angle = 0;

/** @brief Class to implement Background Segmentation
 * 
 *  The class removes the background, associated to the current Kinect angle, from the raw Point Cloud acquired by the Kinect and published on /camera/depth/points.
 * 	The filtered point cloud is published on the topic /camera/pcl_background_segmentation
 */
class PclBackgroundSegmentation{
    /** Handler:
     * - subscribe to /camera/depth/points to get raw data acquired by the Kinect
     * - subscribe to /cur_tilt_angle to know the current tilt angle of the Kinect
     * - publish filtered point cloud on /camera/pcl_background_segmentation
     */
    public:
    PclBackgroundSegmentation(){
        pcl_sub = nh.subscribe("/camera/depth/points", 10, &PclBackgroundSegmentation::backCB, this);
        angle_sub = nh.subscribe("/cur_tilt_angle", 10, &PclBackgroundSegmentation::angleCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/pcl_background_segmentation", 1);
    }
    /**
     * Point cloud callback function:
     *  - remove the background associated to the current Kinect angle from the /camera/depth/points
     *  - publish filtered points cloud in /camera/pcl_background_segmentation
     * @param[in]	input	point cloud data from the Kinect
     */
    void backCB(const sensor_msgs::PointCloud2& input){
        pcl::fromROSMsg(input, *actualImage);

		// Set pointcloud to NAN to remove it
		int index = (angle + 30) / ((int)granularity);
        for (size_t i = 0; i < actualImage->points.size (); ++i){
                if( abs(actualImage->points[i].z - backgrounds[index]->points[i].z) < delta){
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
     * acquires the current tilt angle of the Kinect that is useful to know the right background to remove
     * @param[in]  angle_msg	current tilt angle of the Kinect
     */
    void angleCB(const std_msgs::Float64& angle_msg){
        if(angle_msg.data <= 30.0 && angle_msg.data >= -30.0)
        {
                angle = floor(angle_msg.data);
        }
    }

protected:
	sensor_msgs::PointCloud2 output;
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;  /**< Subscriber to /camera/depth/points */
    ros::Subscriber angle_sub;  /**< Subscriber to /cur_tilt_angle */
    ros::Publisher pcl_pub;  /**< Publisher of filtered point cloud on /camera/pcl_background_segmentation */
};

/**
 * Main function:
 * - initialize of the parameter delta and the handler
 * - acquire and save all backgrounds, associated to different angles of the Kinect
 * @param[in]  delta	sensitivity of the segmentation
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_background_segmentation");
    ros::NodeHandle n("~");
    n.param<double>("delta", delta, 0.08);
    n.param<int>("granularity", granularity, 1);
    
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
            homedir = getpwuid(getuid())->pw_dir;
    }

	// Read backgrounds
    std::string prefix = "/.kinect_environments/environment";
    std::string suffix = ".pcd";
    std::string name;
    
    int index = 0;
    for(int i = -30; i <= 30; i+=granularity)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr curObj (new pcl::PointCloud<pcl::PointXYZ>);
        index = (i + 30) / granularity;
        backgrounds[index] = curObj;
        name = homedir + prefix + std::to_string(i) + suffix;
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (name, *(backgrounds[index])) == -1) {
                PCL_ERROR ("Couldn't read file");
                return (-1);
        }
    }
    
    ROS_INFO ("Finish Loading Environments");

    PclBackgroundSegmentation handler;
    ros::spin();

    return 0;
}
