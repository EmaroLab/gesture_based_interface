#include "BaxterGBI_gui/configpanel.h"
#include "BaxterGBI_gui/mainwindow.h"
#include "BaxterGBI_core_msgs/status.h"
#include <QApplication>
#include "ros/ros.h"
#include <QDebug>

void updatePage(const boost::shared_ptr<BaxterGBI_core_msgs::status> msg){
  ROS_INFO("Context type: [%s]", msg->context_type.c_str());
  qInfo() << "Context type: " << msg->context_type.c_str();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "gui");
    MainWindow w; //instantiate main window
    w.show(); // show main window
    //ConfigPanel w; //instantiate main window
    //w.show(); // show main window

    return a.exec();
}
