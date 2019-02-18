#include "KeystrokePublisher.h"
#include "baxter_gbi_input_msgs/signal.h"

KeystrokePublisher::KeystrokePublisher(QString topic){
    publisher = n.advertise<baxter_gbi_input_msgs::signal>(topic.toStdString(), 1);
}

void KeystrokePublisher::operator()(){
  msg.device_id = "123";
	msg.device_type = "Keyboard";
	msg.device_model = "PC";
	msg.action_descr = "Key pressed";
	msg.confidence = 1;
	publisher.publish(msg);
	ros::spinOnce();
}

