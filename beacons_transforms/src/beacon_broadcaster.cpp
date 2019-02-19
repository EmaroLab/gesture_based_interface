#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include "beacons_transforms/MoveBeacon.h"
#include "beacons_transforms/SetRadius.h"
#include "beacons_transforms/BeaconRadius.h"
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <string>
#include <visualization_msgs/Marker.h>
/**
 * @file
 */
 

/** @brief Class to set a flag when presence is detected
 * 
 */
class Beacon_Radius{
    /** Handler:
     * - subscribe to /beacon_<color>/presence to know when the beacon has detected presence
     */
  public:
    
	bool presence; /**< Flag to signal that presence is detected */
	
	Beacon_Radius(std::string beacon_color){
		std::stringstream beacon_full_sub;
		beacon_full_sub << "/beacon/" << beacon_color << "/presence";
		sub = nh.subscribe(beacon_full_sub.str(), 10, &Beacon_Radius::beaconCB, this);
		presence = false;
	}
	/**
	 * Callback function to set presence flag to true
	 */
	void beaconCB(const std_msgs::Empty& input){
		presence = true;
	}
	/**
	 * Metod to reset the presence flag to false
	 */
	void reset(){
		presence = false;
	}

  protected:
    ros::NodeHandle nh;
    ros::Subscriber sub;  /**< Subscriber to /beacon_<color>/presence */
};

double x_beacon = 0.0; /**<x coordinate of the beacon with respect to the control board */
double y_beacon = 0.0; /**<y coordinate of the beacon with respect to the control board */
double z_beacon = 0.0; /**<z coordinate of the beacon with respect to the control board */

double radius_presence = 1.0; /**<Radius of the circle into which the beacon signals presence */

/**
 * Service to change the position of the beacon
 */
bool move(beacons_transforms::MoveBeacon::Request  &req,
         beacons_transforms::MoveBeacon::Response &res)
{
	x_beacon = req.x;
	y_beacon = req.y;
	z_beacon = req.z;
	res.result = true;
	return true;
}
/**
 * Service to set the radius in which the beacon signals presence
 */
bool set_radius(beacons_transforms::SetRadius::Request  &req,
         beacons_transforms::SetRadius::Response &res)
{
	radius_presence = req.radius;
	
	res.result = true;
	return true;
}
/**
 * Function to assign a unique hash to each beacon
 */
int color_hash(std::string c){
	int ret = 0;
	
	for (int i = 0; i < c.length(); i++)
	{
		ret += (int) (c.at(i) - '0');
	}
	
	return ret;
}

/**
 * Main:
 * - initialization of the parameters
	 * @param[in]  x_beacon  x coordinate of the beacon with respect to the control board
	 * @param[in]  y_beacon    y coordinate of the beacon with respect to the control board
	 * @param[in]  z_beacon    z coordinate of the beacon with respect to the control board
	 * @param[in]  radius_presence    radius into which the beacon signals presence
	 * @param[in]  beacon_color    color which identifies the beacon
	 * @param[in]  beacon_hash    hash of the beacon 
 * 
 * - advertise Services:
	* move_beacon: to move the position of the beacon
	* set_beacon_radius: to set the radius into which the beacon signals presence
 *
 * - publish on /beacon_radius the current radius into which the beacon detects presence
 * - publish on /beacon_radius_rviz to display on rviz of circle into which the beacon signals presence
 */
main(int argc, char** argv)
{
    ros::init(argc, argv, "beacon_broadcaster");
	ros::NodeHandle n("~");
	
	// position of the beacons with respect to the control board
	n.param<double>("x_beacon", x_beacon, 0.0);
	n.param<double>("y_beacon", y_beacon, 0.0);
	n.param<double>("z_beacon", z_beacon, 0.0);
	
	n.param<double>("radius_presence", radius_presence, 1.0);
	
	std::string beacon_color = "color";
	std::string beacon_hash = "00FF00";
	n.param<std::string>("beacon_color", beacon_color, "color");
	n.param<std::string>("beacon_hash", beacon_hash, "00FF00");
    
    Beacon_Radius beacon_radius(beacon_color);
    
	std::stringstream beacon_full_name;
	beacon_full_name << "beacon/" << beacon_color;
	
	// advertise services
	ros::ServiceServer service = n.advertiseService("move_beacon", move);
	ros::ServiceServer service2 = n.advertiseService("set_beacon_radius", set_radius);
	
	ros::Rate r(100);
	tf::TransformBroadcaster broadcaster;
	
    tf::Transform change_frame(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x_beacon, y_beacon, z_beacon));
    
	ros::Publisher beacon_pub = n.advertise<beacons_transforms::BeaconRadius>("beacon_radius", 10);
	
	ros::Publisher beacon_radius_rviz = n.advertise<visualization_msgs::Marker>("beacon_radius_rviz", 1);
	
	beacons_transforms::BeaconRadius msg;
	msg.beacon_name = beacon_full_name.str();
	
	// Display on rviz of circle in which the beacon signals presence
	visualization_msgs::Marker msg_for_rviz_visualization;
	msg_for_rviz_visualization.header.frame_id = beacon_full_name.str();
	msg_for_rviz_visualization.ns = "beacons";
	msg_for_rviz_visualization.id = color_hash(beacon_color);
	msg_for_rviz_visualization.type = visualization_msgs::Marker::CYLINDER;
	msg_for_rviz_visualization.action = visualization_msgs::Marker::ADD;

	msg_for_rviz_visualization.pose.position.x = 0;
	msg_for_rviz_visualization.pose.position.y = 0;
	msg_for_rviz_visualization.pose.orientation.x = 0.0;
	msg_for_rviz_visualization.pose.orientation.y = 0.0;
	msg_for_rviz_visualization.pose.orientation.z = 0.0;
	msg_for_rviz_visualization.pose.orientation.w = 1.0;

	int r_beacon, g_beacon, b_beacon;
	sscanf(beacon_hash.c_str(), "%02x%02x%02x", &r_beacon, &g_beacon, &b_beacon);
	
	msg_for_rviz_visualization.color.a = 0.2; 		//Opacity
	msg_for_rviz_visualization.color.r = r_beacon;
	msg_for_rviz_visualization.color.g = g_beacon;
	msg_for_rviz_visualization.color.b = b_beacon;
	
	//only if using a MESH_RESOURCE marker type:
	msg_for_rviz_visualization.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

	while(ros::ok()){
		msg.radius = radius_presence;
		beacon_pub.publish(msg);
		
		// light the circle of the beacon when presence is detected
		if(beacon_radius.presence){
			msg_for_rviz_visualization.color.a = 0.8; 		//Opacity
		}
		else{
			msg_for_rviz_visualization.color.a = 0.2; 		//Opacity
		}
		msg_for_rviz_visualization.pose.position.z = -z_beacon;
		msg_for_rviz_visualization.header.stamp = ros::Time();
		msg_for_rviz_visualization.scale.x = radius_presence;
		msg_for_rviz_visualization.scale.y = radius_presence;
		msg_for_rviz_visualization.scale.z = 0.1; //height of the cilinder
		beacon_radius_rviz.publish(msg_for_rviz_visualization);
		
		// Trasformation from world frame to beacon frame
		change_frame.setOrigin(tf::Vector3(x_beacon, y_beacon, z_beacon));
		broadcaster.sendTransform(tf::StampedTransform(change_frame, ros::Time::now(), "world_frame", beacon_full_name.str()));
		
		// Metod to reset the presence flag to false
		beacon_radius.reset();
		
		ros::spinOnce();
		r.sleep();
	}
  
  return 0;
}
