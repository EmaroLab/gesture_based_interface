#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <math.h>
#include <Eigen/Geometry> 

float angle = -30.0;
int go = 0;

class cloudHandler{
	public:
    	cloudHandler(){
        pcl_sub = nh.subscribe("/camera/depth/points", 10, &cloudHandler::cloudCB, this);
    }

    
	void cloudCB (const sensor_msgs::PointCloud2ConstPtr& cloud)
	{
		if ((cloud->width * cloud->height) == 0)
			return;
			
		if(go == 0)
			return;
		go = 0;

		int print_angle = angle;
		std::stringstream ss;
		ss << prefix_ << print_angle << ".pcd";
		ROS_INFO ("Data saved to %s", ss.str ().c_str ());

		pcl::io::savePCDFile (ss.str (), *cloud, Eigen::Vector4f::Zero (),
						 Eigen::Quaternionf::Identity (), false);
	}

	protected:
		ros::NodeHandle nh;
		ros::Subscriber pcl_sub;
		ros::Subscriber angle_sub;
		char* prefix_ = "/home/marco/catkin_ws/src/pose_estimation/src/environments/environment";

};


int main(int argc, char **argv)
{
ros::init(argc, argv, "recorder");

cloudHandler handler;

ros::NodeHandle n;

ros::Publisher rec_pub = n.advertise<std_msgs::Float64>("tilt_angle", 1000);

ros::Rate loop_rate(0.3);
ros::Rate sleep1(0.2);

angle = -30.0;
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
	
	angle = angle + 1.0;
	
	if(angle >= 31.0)
		exit(0);
}


return 0;
}
