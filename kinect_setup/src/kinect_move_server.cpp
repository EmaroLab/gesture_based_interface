#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "kinect_setup/MoveKinect.h"

/** 
 * Publisher, to regulate the tilt angle of the Kinect
 */
ros::Publisher rec_pub;

/** 
 * Message to publish
 */
std_msgs::Float64 msg;

/** Callback of the move_kinect service
  * @param[in] req request message
	* @param[in] req.angle desired orientation angle of the Kinect
  * @param[out] res response of the service
	* @param[out] res.result check if the operation is completed successfully
  */
bool move(kinect_setup::MoveKinect::Request  &req,
         kinect_setup::MoveKinect::Response &res){
	// Saturate angle if it exceeds boudaries
	if(req.angle > 30){
		req.angle = 30;
	}
	if(req.angle < -30){
		req.angle = -30;
	}
	
	msg.data = req.angle;
	
	// Publish message on /tilt_angle topic (the kinect_aux_node will move the Kinect)
	rec_pub.publish(msg);
	
	res.result = true;
	return true;
}
/**
 * Main:
 * Initialization of the service kinect_move_server and the publisher
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "kinect_move_server");
	ros::NodeHandle n;
	
	// Initialize Publisher
	rec_pub = n.advertise<std_msgs::Float64>("tilt_angle", 1000);
	
	// Advertise Service
	ros::ServiceServer service = n.advertiseService("move_kinect", move);
  ros::spin();
  return 0;
}
