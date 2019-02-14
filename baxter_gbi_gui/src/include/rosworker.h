#ifndef ROSWORKER_H
#define ROSWORKER_H

#include "baxter_gbi_core_msgs/status.h"
#include "ros/ros.h"

#include <QObject>
#include <QVector>
#include <QString>

class Worker : public QObject{
	Q_OBJECT
    
private:
	ros::Subscriber sub;
	ros::NodeHandle n;
    
public:
	explicit Worker(QWidget *parent = 0);
	~Worker();

public slots:
	void start();
	void stop();
    
signals:
    void configFrame();
    void menuFrame(QString &title,
                   QVector<QString> &options,
                   QVector<QString> &fixed_options,
                   int8_t selection);
    void actionFrame(QString action, 
                     QString msg);
	void finished();

private:
	void statusCb(const boost::shared_ptr<baxter_gbi_core_msgs::status>);
};

#endif //ROSWORKER_H
