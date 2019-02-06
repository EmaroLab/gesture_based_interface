#include "KeystrokePublisher.h"
#include "baxter_gbi_input_msgs/signal.h"

KeystrokePublisher::KeystrokePublisher(std::string topic)
: topic(topic){
    publisher = n.advertise<baxter_gbi_input_msgs::signal>(topic, 1);
}

KeystrokePublisher::KeystrokePublisher(QString topic)
: KeystrokePublisher(topic.toStdString())
{}

void KeystrokePublisher::operator()(){
  msg.device_id = "123";
	msg.device_type = "Keyboard";
	msg.device_model = "PC";
	msg.action_descr = "Key pressed";
	msg.confidence = 1;
	publisher.publish(msg);
	ros::spinOnce();
}

