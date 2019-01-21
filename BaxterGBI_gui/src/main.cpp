#include "BaxterGBI_gui/configpanel.h" //include our header
#include "BaxterGBI_gui/mainwindow.h" //include our header
#include <QApplication>
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "topic_scanner");
    ros::NodeHandle n;
    MainWindow w; //instantiate main window
    w.show(); // show main window
    //ConfigPanel w; //instantiate main window
    //w.show(); // show main window

    return a.exec();
}
