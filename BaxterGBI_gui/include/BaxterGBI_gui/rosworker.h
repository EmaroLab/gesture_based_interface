#ifndef ROSWORKER_H
#define ROSWORKER_H

#include "BaxterGBI_core_msgs/status.h"
#include "ros/ros.h"

#include <QObject>

class Worker : public QObject{
	Q_OBJECT
    
private:
	ros::Subscriber sub;
	ros::NodeHandle n;
    
public:
	explicit Worker(QWidget *parent = 0);
	~Worker();

public slots:
	void process();
	void stop();
    
signals:
	void newStatus(const boost::shared_ptr<BaxterGBI_core_msgs::status>);
	void finished();
    
private:
	void statusCb(const boost::shared_ptr<BaxterGBI_core_msgs::status>);
};

#endif //ROSWORKER_H
