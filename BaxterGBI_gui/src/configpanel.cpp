#include "BaxterGBI_gui/configpanel.h"
#include "ui_configpanel.h"

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

ConfigPanel::ConfigPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ConfigPanel),
    model(new QStandardItemModel)
{
	qInfo() << "A" << model;
    ui->setupUi(this);
    ui->tabWidget->clear();
    for (int i = 1; i <= 6; i++)
        ui->tabWidget->addTab(new TabContent(model), QString("Action %1").arg(i));

    connect(ui->scanButton, &QPushButton::clicked, this, &ConfigPanel::scan);
}

void ConfigPanel::scan(){
	ros::master::V_TopicInfo topicMap;
	std::regex regex("^\\/([a-zA-Z][0-9a-zA-Z_]*)\\/([0-9a-zA-Z_]+)$"); //to match topics
	std::smatch match;
	ros::master::getTopics(topicMap);

	for (int i = 0; i <= 5; i++){
		static_cast<TabContent*>(ui->tabWidget->widget(i))->clear();
		static_cast<TabContent*>(ui->tabWidget->widget(i))->enableAddButton();
	}
  
  compatibleSubtopics.clear(); //erases all elements from the container
  model->clear();

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
  
  for(auto a: compatibleSubtopics){
	  QStandardItem* topic = new QStandardItem(a.first.c_str());
	  model->appendRow(topic);
	  for(auto b: a.second){
		  QStandardItem* subtopic = new QStandardItem(b.c_str());
		  topic->appendRow(subtopic);
	  }
  }
  
  for (int i = 0; i < model->rowCount(); i++){
	qInfo() << model->item(i,0)->text();
	for (int j = 0; j < model->item(i,0)->rowCount(); j++){
		qInfo() << '\t' << model->item(i,0)->child(j,0)->text();
	}
  }	
}

ConfigPanel::~ConfigPanel(){
    delete ui;
}
