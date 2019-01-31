#include <ros/ros.h>
#include "std_msgs/String.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "pose_estimation/SetFilter.h"
#include "pose_estimation/SetFilterParam.h"
/**
 * @file
 */
 
// Filter parameters
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

/** Callback function of enable_service
 * for enabling filters.
 */
bool set_filter(pose_estimation::SetFilter::Request  &req,
         pose_estimation::SetFilter::Response &res)
{
	std::string str_z = "z_filter";
	std::string str_y = "y_filter";
	std::string str_x = "x_filter";
	std::string str_down = "downsampling_filter";
	std::string str_sor = "sor_filter";
	if(req.filter_name.compare(str_z) == 0)
	{
		use_filter_z = req.enable;
	}
	else if(req.filter_name.compare(str_y) == 0)
	{
		use_filter_y = req.enable;
	}
	else if(req.filter_name.compare(str_x) == 0)
	{
		use_filter_x = req.enable;
	}
	else if(req.filter_name.compare(str_down) == 0)
	{
		use_downsampling = req.enable;
	}
	else if(req.filter_name.compare(str_sor) == 0)
	{
		use_filter_sor = req.enable;
	}
	res.result = true;
	return true;
}

/** Callback function of a service
 * to set filters parameters (leaf_size, range of XYZ filter, SOR filter parameters).
 */
bool set_filter_param(pose_estimation::SetFilterParam::Request  &req,
         pose_estimation::SetFilterParam::Response &res)
{
	std::string str_leaf = "leaf_size";
	std::string str_min_z = "min_z";
	std::string str_max_z = "max_z";
	std::string str_min_y = "min_y";
	std::string str_max_y = "max_y";
	std::string str_min_x = "min_x";
	std::string str_max_x = "max_x";
	std::string str_sor_k = "sor_k";
	std::string str_sor_stddev = "sor_stddev";
	
	std::string str_revert_x = "revert_x";
	std::string str_revert_y = "revert_y";
	std::string str_revert_z = "revert_z";
	
	if(req.param_name.compare(str_leaf) == 0)
	{
		leaf_size = req.value;
	}
	else if(req.param_name.compare(str_min_z) == 0)
	{
		min_z = req.value;
	}
	else if(req.param_name.compare(str_max_z) == 0)
	{
		max_z = req.value;
	}
	else if(req.param_name.compare(str_min_y) == 0)
	{
		min_y = req.value;
	}
	else if(req.param_name.compare(str_max_y) == 0)
	{
		max_y = req.value;
	}
	else if(req.param_name.compare(str_min_x) == 0)
	{
		min_x = req.value;
	}
	else if(req.param_name.compare(str_sor_k) == 0)
	{
		sor_k = (int) req.value;
	}
	else if(req.param_name.compare(str_sor_stddev) == 0)
	{
		sor_stddev = req.value;
	}
	else if(req.param_name.compare(str_revert_x) == 0)
	{
		revert_filter_x = req.value;
	}
	else if(req.param_name.compare(str_revert_y) == 0)
	{
		revert_filter_y = req.value;
	}
	else if(req.param_name.compare(str_revert_z) == 0)
	{
		revert_filter_z = req.value;
	}
	
	res.result = true;
	return true;
}
/**@brief Class to filter the Point Cloud
 * 
 * The class has the aim of filtering the Point Cloud obtained by the background segmentation and published on /camera/pcl_background_segmentation.
 * It uses different filters:
 * 		- Downsampling (with VoxelGrid)
 * 		- XYZ axes range filter
 * 		- Statistical Outlier Removal Filter
 * and publish the filtered point cloud on /camera/pcl_filtered
 */
class PclFilter
{
private:

	/** 
	 * Input Point Cloud
	 */
	pcl::PointCloud<pcl::PointXYZ> cloud;
	/** 
	 * Output Point Cloud
	 */
	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
	/** 
	 * Statistical Outlier Removal Filter
	 */
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;

public:
	/** Handler:
	 * - subscribe to /camera/pcl_background_segmentation to get the point cloud obtained by the background segmentation
	 * - publish filtered point cloud on /camera/pcl_filtered 
	 */
    PclFilter()
    {
        pcl_sub = nh.subscribe("/camera/pcl_background_segmentation", 100, &PclFilter::filterCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/pcl_filtered", 100);
    }
	/** 
	 * Callback function
	 * implementation of filters:
	 * - Downsampling filter with parameter leaf_size
	 * - Filter on z axis in range (min_z, max_z)
	 * - Filter on x axis in range (min_x, max_x)
	 * - Filter on y axis in range (min_y, max_y)
	 * - Statistical Outlier Removal filter to remove outliers 
	 * @param[in] input point cloud data from background segmentation
	 */
    void filterCB(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
    {
		pcl::PCLPointCloud2::Ptr input_pcl (new pcl::PCLPointCloud2 ());
		pcl_conversions::toPCL(*input, *input_pcl);
		
		pcl::PCLPointCloud2::Ptr input_pcl_filtered (new pcl::PCLPointCloud2 ());
		
		// Downsampling filter with parameter leaf_size
		if(use_downsampling)
		{
			voxel.setInputCloud (input_pcl);
			voxel.setLeafSize (leaf_size, leaf_size, leaf_size);
			voxel.filter (*input_pcl_filtered);
		}
		else
		{
			input_pcl_filtered = input_pcl;
		}
			
		pcl::fromPCLPointCloud2(*input_pcl_filtered, cloud);
		
		if(!cloud.empty()){	
			// Filter on z axis in range (min_z, max_z)		
			if(use_filter_z)
			{
				pass_z.setInputCloud (cloud.makeShared());
				pass_z.setFilterFieldName ("z");
				pass_z.setFilterLimits (min_z, max_z);
				
				// To use the complementar range
				if(revert_filter_z)
					pass_z.setFilterLimitsNegative (true);
				else
					pass_z.setFilterLimitsNegative (false);
				
				pass_z.filter (cloud_filtered);
			}
			if(!cloud_filtered.empty()){
				// Filter on x axis in range (min_x, max_x)
				if(use_filter_x)
				{
					pass_x.setInputCloud (cloud_filtered.makeShared());
					pass_x.setFilterFieldName ("x");
					pass_x.setFilterLimits (min_x, max_x);
					
					// To use the complementar range
					if(revert_filter_x)
						pass_x.setFilterLimitsNegative (true);
					else
						pass_x.setFilterLimitsNegative (false);
					
					pass_x.filter (cloud_filtered);
				}
				
				if(!cloud_filtered.empty()){
					// Filter on y axis in range (min_y, max_y)
					if(use_filter_y)
					{
						pass_y.setInputCloud (cloud_filtered.makeShared());
						pass_y.setFilterFieldName ("y");
						pass_y.setFilterLimits (min_y, max_y);
						
						// To use the complementar range
						if(revert_filter_y)
							pass_y.setFilterLimitsNegative (true);
						else
							pass_y.setFilterLimitsNegative (false);
					
						pass_y.filter (cloud_filtered);
					}
					if(!cloud_filtered.empty()){
						// Removing outliers using a StatisticalOutlierRemoval filter
						if(use_filter_sor){
							sor.setInputCloud (cloud_filtered.makeShared());
							sor.setMeanK (sor_k);
							sor.setStddevMulThresh (sor_stddev);
							sor.filter (cloud_filtered);
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
    ros::NodeHandle nh;		/**< Node Handler */
    ros::Subscriber pcl_sub; /**< Subscriber to /camera/pcl_background_segmentation */
    ros::Publisher pcl_pub; /**< Publisher of filtered point cloud on /camera/pcl_filtered */
	sensor_msgs::PointCloud2 output;		/**< Output Message to publish */
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;		/**< SOR Filter */
	pcl::PassThrough<pcl::PointXYZ> pass_x;		/**< X Range Filter */
	pcl::PassThrough<pcl::PointXYZ> pass_y;		/**< Y Range Filter */
	pcl::PassThrough<pcl::PointXYZ> pass_z;		/**< Z Range Filter */
	pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;		/**< VoxelGrid Filter */
};

/**
 * Main:
 * Initialization of the parameters and of the handler
 * @param[in]  use_downsampling  Enable downsampling filter
 * @param[in]  downsampling_leaf_size    Downsampling granularity
 * @param[in]  use_filter_z    Enable z axis filter
 * @param[in]  min_z    left limit of z range
 * @param[in]  max_z    right limit of z range
 * @param[in]  revert_filter_z    if true use the complementar range 
 * @param[in]  use_filter_y    Enable y axis filter
 * @param[in]  min_y    left limit of y range
 * @param[in]  max_y    right limit of y range
 * @param[in]  revert_filter_y    if true use the complementar range  
 * @param[in]  use_filter_x    Enable x axis filter
 * @param[in]  min_x    left limit of x range
 * @param[in]  max_x    right limit of x range
 * @param[in]  revert_filter_x    If true use the complementar range 
 * @param[in]  use_filter_sor    Enable SOR filter
 * @param[in]  sor_k    Parameter of SOR filter
 * @param[in]  sor_stddev    Parameter of SOR filter
 * 
 * Advertise Services:
	* set_filter: to enable/disable filters
	* set_filter_param: to set a parameter of one filter
 */
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
    
    PclFilter handler;

	ros::ServiceServer enable_service = n.advertiseService("set_filter", set_filter);
	ros::ServiceServer service = n.advertiseService("set_filter_param", set_filter_param);
	
    ros::spin();

    return 0;
}
