#include "BaxterGBI_gui/configpanel.h"
#include "ui_configpanel.h"

#include <string>
#include <map>
#include <vector>
#include <regex>
#include <initializer_list>
#include <algorithm>

#include "ros/ros.h"
#include "ros/master.h"

#include <QDebug>
#include <QInputDialog>

ConfigPanel::ConfigPanel(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::ConfigPanel),
	model(new QStandardItemModel),
	isFilled(6, false) //inizialize elements to false
{
	ui->setupUi(this);
	ui->loadConfigButton->setEnabled(false); //disable load button
	ui->tabWidget->clear();
	
	ui->addMappingButton->setEnabled(false); //disable add button

	for (int i = 0; i < 6; i++){
		tabs[i] = new TabContent(model);
		ui->tabWidget->addTab(tabs[i], QString("Action %1").arg(i+1));
		connect(tabs[i], &TabContent::numberOfMappings, 
						[=](const int &mappings){enableLoadButton(i, mappings);
						});
	}
	connect(ui->scanButton, &QPushButton::clicked, this, &ConfigPanel::scan);
	connect(ui->addMappingButton, &QPushButton::clicked, this, &ConfigPanel::addMappingToActiveTab);
	connect(ui->loadConfigButton, &QPushButton::clicked, this, &ConfigPanel::sendConfig);
}

ConfigPanel::~ConfigPanel(){
	delete ui;
}

void ConfigPanel::scan(){
  ros::master::V_TopicInfo topicMap;
  std::regex regex("^\\/([a-zA-Z][0-9a-zA-Z_]*)\\/([0-9a-zA-Z_]+)$"); //to match topics
  std::smatch match;
  ros::master::getTopics(topicMap);
  int n_topics = 0;

  for (int i = 0; i < 6; i++){
    tabs[i]->clear();
	}
  ui->addMappingButton->setEnabled(false);
  
  compatibleSubtopics.clear(); //erases all elements from the container
  model->clear();

  for (ros::master::TopicInfo ti: topicMap){
    if (ti.datatype == "BaxterGBI_input_msgs/signal"){
      if (std::regex_match(ti.name, match, regex)){
        ++n_topics;
        static std::string topic, subtopic;
        topic = match[1];
        subtopic = match[2];
        auto [__discard, first_sub] = compatibleSubtopics.try_emplace(topic, std::initializer_list<std::string>{subtopic});
        if (not first_sub)
          compatibleSubtopics[topic].insert(std::upper_bound(compatibleSubtopics[topic].begin(),
                                                             compatibleSubtopics[topic].end(),
                                                             subtopic),
                                            subtopic);
      }
    }
  }

  for(auto a: compatibleSubtopics){
    qInfo() << "Topic %s:" << a.first.c_str();
    ROS_INFO("Topic %s:", a.first.c_str());
    auto topic = new QStandardItem(a.first.c_str());
    model->appendRow(topic);
	  for(auto b: a.second){
      qInfo() << "\t%s:" << b.c_str();
      ROS_INFO("\t%s", b.c_str());
      topic->appendRow(new QStandardItem(b.c_str()));
	  }
  }
	ui->addMappingButton->setEnabled(model->rowCount());
}

void ConfigPanel::addMappingToActiveTab(){
	tabs[ui->tabWidget->currentIndex()]->addMapping();
}

void ConfigPanel::enableLoadButton(int tab, int mappings){
	isFilled[tab] = mappings > 0; //set array element to true
	
  bool ok = true;
  for(int i = 0; i < 6 and ok; i++)
    ok = isFilled[i];
	
  ui->loadConfigButton->setEnabled(ok);
}

void ConfigPanel::sendConfig(){
	for(int i = 0; i < 6; i++){
		auto selections = tabs[i]->getSelectedTopics();
		qInfo() << "Tab " << i << " mappings: ";
		std::vector<std::string> serializedTopics;
		 
		for(auto selection : selections){
			QString topic = QString("/%1/%2").arg(selection.first).arg(selection.second);
			serializedTopics.push_back(topic.toStdString());
			qInfo() << "\t" << topic;
		}
		ros::NodeHandle n;
		n.setParam("key_" + std::to_string(i+1) + "_topics", serializedTopics);
	}
	/*
	ros::ServiceClient client = nh.serviceClient<my_package::Foo>("/fsm_config");
	my_package::Foo foo;
	client.call(foo)
	// call service /fsm_config of type std_srvs/Trigger
	* */
}
