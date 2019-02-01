#include "rosworker.h"
#include "ros/ros.h"

#include <QMetaType>
#include <QDebug>
#include <QThread>

#include <string>
#include <vector>


QVector<QString> cxx2qt_strvec(std::vector<std::string> &stdvstds){
  std::vector<QString> v;
  v.clear();
  v.reserve(stdvstds.size());
  std::transform(stdvstds.begin(),
                 stdvstds.end(),
                 std::back_inserter(v),
                 QString::fromStdString);
  return QVector<QString>::fromStdVector(v);
}

Worker::Worker(QWidget* parent){
}

Worker::~Worker(){
}

void Worker::statusCb(const boost::shared_ptr<BaxterGBI_core_msgs::status> msg){
	if(msg->context_type == "config_wait" or msg->context_type == "wait_user"){
		emit configFrame();
	} else if (msg->context_type == "menu"){
		auto title = QString::fromStdString(msg->m_title);
		auto options = cxx2qt_strvec(msg->m_options);
		auto fixed_options = cxx2qt_strvec(msg->m_fixed_options);
		emit menuFrame(title, options, fixed_options, msg->m_selection);
	} else if (msg->context_type == "action"){
		auto action = QString::fromStdString(msg->pbr_action);
		auto message = QString::fromStdString(msg->pbr_msg);
		emit actionFrame(action, message);
	} else {
		ROS_WARN("Wrong context type in received status msg.");
	};
}

void Worker::start(){
	sub = n.subscribe("/fsm_status", 10, &Worker::statusCb, this);
	ros::spin();
	emit finished();
}

void Worker::stop(){
	ros::shutdown();
}
