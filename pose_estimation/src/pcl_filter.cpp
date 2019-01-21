#include <ros/ros.h>
#include "std_msgs/String.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

//Use this class to contain a public member that is used in the callback function

bool use_downsampling = true;
bool use_filter_z = true;
bool use_filter_y = true;
bool use_filter_x = true;
bool use_filter_sor = true;

bool revert_filter_z = false;
bool revert_filter_y = false;
bool revert_filter_x = false;

double leaf_size = 0.1;

double min_z = 0.1;
double max_z = 3;

double min_y = -0.5;
double max_y = 0.5;

double min_x = -0.5;
double max_x = 0.5;

int sor_k = 40;
double sor_stddev = 1.0;


class cloudHandler
{
private:
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;

public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("/camera/pcl_background_segmentation", 100, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/pcl_filtered", 100);
    }

    void cloudCB(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
    {
		pcl::PCLPointCloud2::Ptr input_pcl (new pcl::PCLPointCloud2 ());
		pcl_conversions::toPCL(*input, *input_pcl);
		
		pcl::PCLPointCloud2::Ptr input_pcl_filtered (new pcl::PCLPointCloud2 ());
		
		if(use_downsampling)
		{
			// Create the filtering object
			pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
			sor.setInputCloud (input_pcl);
			sor.setLeafSize (leaf_size, leaf_size, leaf_size);
			sor.filter (*input_pcl_filtered);
		}
		else
		{
			input_pcl_filtered = input_pcl;
		}
			
		pcl::fromPCLPointCloud2(*input_pcl_filtered, cloud);
        sensor_msgs::PointCloud2 output;
		
		if(!cloud.empty()){			
			if(use_filter_z)
			{
				//passThrough Filter 
				pcl::PassThrough<pcl::PointXYZ> pass;
				pass.setInputCloud (cloud.makeShared());
				pass.setFilterFieldName ("z");
				pass.setFilterLimits (min_z, max_z);
				
				if(revert_filter_z)
					pass.setFilterLimitsNegative (true);
					
				pass.filter (cloud_filtered);
			}
			if(!cloud_filtered.empty()){
				if(use_filter_x)
				{
					pcl::PassThrough<pcl::PointXYZ> pass2;
					pass2.setInputCloud (cloud_filtered.makeShared());
					pass2.setFilterFieldName ("x");
					pass2.setFilterLimits (min_x, max_x);
					
					if(revert_filter_x)
						pass2.setFilterLimitsNegative (true);
					
					pass2.filter (cloud_filtered);
				}
				
				if(!cloud_filtered.empty()){
					if(use_filter_y)
					{
						pcl::PassThrough<pcl::PointXYZ> pass3;
						pass3.setInputCloud (cloud_filtered.makeShared());
						pass3.setFilterFieldName ("y");
						pass3.setFilterLimits (min_y, max_y);
						
						if(revert_filter_y)
							pass3.setFilterLimitsNegative (true);
					
						pass3.filter (cloud_filtered);
					}
					if(!cloud_filtered.empty()){
						
						if(use_filter_sor){
							// Create the filtering object
							// Removing outliers using a StatisticalOutlierRemoval filter
							pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
							sor2.setInputCloud (cloud_filtered.makeShared());
							sor2.setMeanK (sor_k);
							sor2.setStddevMulThresh (sor_stddev);
							sor2.filter (cloud_filtered);
						}
					}
				}
			}
			pcl::toROSMsg(cloud_filtered, output);
		}
        else
        {
			pcl::toROSMsg(cloud, output);
		}
        pcl_pub.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_filter");
	ros::NodeHandle n("~");
    n.param<double>("min_z", min_z, 0.1);
    n.param<double>("max_z", max_z, 3);
    n.param<double>("min_y", min_y, -0.5);
    n.param<double>("max_y", max_y, 0.5);
    n.param<double>("min_x", min_x, -0.5);
    n.param<double>("max_x", max_x, 0.5);
    
    n.param<int>("sor_k", sor_k, 40);
    n.param<double>("sor_stddev", sor_stddev, 1.0);
    
    n.param<double>("downsampling_leaf_size", leaf_size, 0.01);
    
    n.param<bool>("use_filter_z", use_filter_z, true);
    n.param<bool>("use_filter_y", use_filter_y, true);
    n.param<bool>("use_filter_x", use_filter_x, true);
    n.param<bool>("use_filter_sor", use_filter_sor, true);
    n.param<bool>("use_downsampling", use_downsampling, true);
    
    n.param<bool>("revert_filter_z", revert_filter_z, false);
    n.param<bool>("revert_filter_y", revert_filter_y, false);
    n.param<bool>("revert_filter_x", revert_filter_x, false);
    
    cloudHandler handler;

    ros::spin();

    return 0;
}
