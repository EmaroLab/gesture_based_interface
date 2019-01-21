#include "BaxterGBI_gui/mainwindow.h" //include our header
#include "ui_mainwindow.h" //required by the generated class Ui::MainWindow

#include <QDebug>
#include <QInputDialog>

#include "BaxterGBI_gui/tabcontent.h"

#include <string>
#include <map>
#include <vector>
#include <regex>
#include <initializer_list>
#include "ros/ros.h"
#include "ros/master.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)  //initialize private member variable
{
    ui->setupUi(this); //initialize all widgets used by the MainWindow.ui design file
    ui->tabWidget->clear();
    for (int i = 1; i<=6; i++)
        ui->tabWidget->addTab(new TabContent(), QString("Action %1").arg(i));

    connect(ui->scanButton, &QPushButton::clicked, this, &MainWindow::scan);
}

void MainWindow::scan(){
  ROS_INFO("Ready to scan.");

  ros::master::V_TopicInfo topicMap;
  std::map<std::string, std::vector<std::string>> compatibleSubtopics;
  std::regex regex("^\\/([a-zA-Z][0-9a-zA-Z_]*)\\/([0-9a-zA-Z_]+)$");
  std::smatch match;
  ros::master::getTopics(topicMap);

  for (ros::master::TopicInfo ti: topicMap){
    if (ti.datatype == "BaxterGBI_input_msgs/signal"){
      ROS_INFO("%s | %s", ti.name.c_str(), ti.datatype.c_str());
      if (std::regex_match(ti.name, match, regex)){
        static std::string topic, subtopic;
        topic = match[1];
        subtopic = match[2];
        ROS_INFO("topic: %s | subtopic: %s", topic.c_str(), subtopic.c_str());
        auto [__discard, first_sub] = compatibleSubtopics.try_emplace(topic, std::initializer_list<std::string>{subtopic});
        if (first_sub){
          ROS_INFO("This was the first subtopic of that topic");
        } else {
          compatibleSubtopics[topic].push_back(subtopic);
        }
      }
    }
  }

  for (auto v: compatibleSubtopics){
    ROS_INFO("Topic %s:", v.first.c_str());  
    for(auto s: v.second){
      ROS_INFO("\t%s", s.c_str());  
    }
  }
}

MainWindow::~MainWindow()
{
    delete ui; //this will bring down the whole QObject hierarchy
}
