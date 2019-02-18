#include <QApplication>

#include "ros/ros.h"
#include "mainwindow.h"

// initialization of the "gui" node
int main(int argc, char *argv[]){
  QApplication a(argc, argv);
  ros::init(argc, argv, "gui");
  MainWindow w;
  w.show();
  return a.exec();
}
