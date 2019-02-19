#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"

#include <sstream>
#include <string.h>
/**
 * @file
 */

void callBack1(const std_msgs::Empty msg);
void callBack2(const std_msgs::Empty msg);
void callBack3(const std_msgs::Empty msg);

void (*p[3])(const std_msgs::Empty msg); /** < array of pointers to the callbacks */
bool presences[3] = {false, false, false}; /** < array of bool where true = presence while false = absence */

/** Main
 * @param[in] beacons_number number of beacons taken from the launchfile
 * @param[in] beacon_color array of colors which identify each beacon
 * @param[in] topics array of topics to which the subscribers subscribe
 * @param[in] subs array of subscribers (one for each beacon) 
 */ 
int main(int argc, char **argv){
	
    ros::init(argc, argv, "beacons_scanner");
    ros::NodeHandle n("~"); 
    int beacons_number; 
    int min_beacons;
    std::string beacon_color[3]; 
    
    std::stringstream topics[3];
    ros::Subscriber subs[3];
    
    p[0] = callBack1;
		p[1] = callBack2; 
		p[2] = callBack3; 
    
    n.param<int>("beacons_number", beacons_number, 2);
    n.param<int>("min_beacons", min_beacons, 2);
    n.param<std::string>("beacon_color_1", beacon_color[0], "pink");
    n.param<std::string>("beacon_color_2", beacon_color[1], "yellow");
    n.param<std::string>("beacon_color_3", beacon_color[2], "violet");
		
		ros::NodeHandle nh;
		for(int i = 0; i < beacons_number; i++){
			topics[i] <<"/beacon/" << beacon_color[i] << "/presence";
			subs[i] = nh.subscribe(topics[i].str(), 1, p[i]);
		}
		
    ros::Publisher pub = nh.advertise<std_msgs::Header>("/beacons/presence", 1); 
    ros::Rate loop_rate(100);
		
		std_msgs::Header msg;
		int count = 0;
		
    while (ros::ok()){
				count = 0;
				for(int i = 0; i < beacons_number; i++){
					if(presences[i]) count++;
					ROS_INFO("%d", count);
					if(count == min_beacons){
						msg.stamp = ros::Time::now();
						pub.publish(msg);
						ROS_INFO("%d", count);
						break;
					}
				}
				
				//reset the elements of presences to false
				for(int i = 0; i < beacons_number; i++)
						presences[i] = false;
				
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/**
 * Callback function which is called when the first beacon detects the 
 * presence of the user
 * @param[in]	msg	empty message sent by the first beacon
 */
void callBack1(const std_msgs::Empty msg){
	ROS_INFO("I heard /beacons/1");
	presences[0] = true;
}

/**
 * Callback function which is called when the second beacon detects the 
 * presence of the user
 * @param[in]	msg	message sent by the second beacon
 */
void callBack2(const std_msgs::Empty msg){
	ROS_INFO("I heard /beacons/2");
	presences[1] = true;
}

/**
 * Callback function which is called when the third beacon detects the 
 * presence of the user
 * @param[in]	msg	message sent by the third beacon
 */
void callBack3(const std_msgs::Empty msg){
  ROS_INFO("I heard /beacons/3");
	presences[2] = true;
}
