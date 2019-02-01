#include "rosworker.h"
#include "ros/ros.h"

#include <QMetaType>
#include <QDebug>
#include <QThread>

Worker::Worker(QWidget* parent){
}

Worker::~Worker(){
}

void Worker::statusCb(const boost::shared_ptr<BaxterGBI_core_msgs::status> msg){
	emit newStatus(msg);
}

void Worker::process(){
	qRegisterMetaType<boost::shared_ptr<BaxterGBI_core_msgs::status>>("boost::shared_ptr<BaxterGBI_core_msgs::status>");
	sub = n.subscribe("/fsm_status", 10, &Worker::statusCb, this);
	ros::spin();
	emit finished();
}

void Worker::stop(){
	ros::shutdown();
}
