#include "configpanel.h"
#include "ui_configpanel.h"

#include <string>
#include <map>
#include <vector>
#include <regex>
#include <initializer_list>
#include <algorithm>

#include "ros/ros.h"
#include "ros/master.h"
#include "std_srvs/Trigger.h"

#include <QDebug>
#include <QInputDialog>

ConfigPanel::ConfigPanel(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::ConfigPanel),
	model(new QStandardItemModel),
	isFilled(6, false) ,
	fsmReconfigure("/fsm_config")
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
        connect(tabs[i], &TabContent::mappingRemoved,
                        [&](){
                              ++n_topics;
                              ui->addMappingButton->setEnabled(n_topics);
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
        static QString topic, subtopic;
        topic = QString::fromStdString(match[1]);
        subtopic = QString::fromStdString(match[2]);
        auto iterator = compatibleSubtopics.find(topic);
        if (iterator != compatibleSubtopics.end()){
            iterator.value().append(subtopic);
        } else {
            compatibleSubtopics.insert(topic, QVector<QString>{subtopic});
        }
      }
    }
  }

  for (auto a = compatibleSubtopics.begin(); a != compatibleSubtopics.end(); ++a){
    qInfo() << "Topic %s:" << a.key();
    ROS_INFO("Topic %s:", a.key().toLatin1().data());
    auto topic = new QStandardItem(a.key());
    model->appendRow(topic);
    for(auto b: a.value()){
      qInfo() << "\t%s:" << b;
      ROS_INFO("\t%s", b.toLatin1().data());
      topic->appendRow(new QStandardItem(b));
    }
  }
  ui->addMappingButton->setEnabled(n_topics >= 6);
}

void ConfigPanel::addMappingToActiveTab(){
  tabs[ui->tabWidget->currentIndex()]->addMapping();
  --n_topics;
  ui->addMappingButton->setEnabled(n_topics);
}


void ConfigPanel::enableLoadButton(int tab, int mappings){
  isFilled[tab] = mappings > 0; //set array element to true
	
  bool ok = true;
  for(int i = 0; i < 6 and ok; i++)
    ok = isFilled[i];
	
  ui->loadConfigButton->setEnabled(ok);
}

void ConfigPanel::sendConfig(){
  ros::NodeHandle n;
  for(int i = 0; i < 6; i++){
    auto selections = tabs[i]->getSelectedTopics();
    qInfo() << "Tab " << i << " mappings: ";
    std::vector<std::string> serializedTopics;

    for(auto selection : selections){
      QString topic = QString("/%1/%2").arg(selection.first).arg(selection.second);
      serializedTopics.push_back(topic.toStdString());
      qInfo() << "\t" << topic;
    }
    n.setParam("key_" + std::to_string(i+1) + "_topics", serializedTopics);
  }

  fsmReconfigure();
}
