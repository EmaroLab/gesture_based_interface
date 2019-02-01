#include "configpanel.h"
#include "ui_configpanel.h"

#include "ros/ros.h"
#include "ros/master.h"

#include <QDebug>
#include <QInputDialog>

ConfigPanel::ConfigPanel(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::ConfigPanel),
	model(new QStandardItemModel),
	isFilled(6, false) ,
	fsmReconfigure("/fsm_config"),
	scanner("BaxterGBI_input_msgs/signal")
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
  for (int i = 0; i < 6; i++){
    tabs[i]->clear();
	}
  ui->addMappingButton->setEnabled(false);
  model->clear();
  scanner();

  for (auto a = scanner.begin(); a != scanner.end(); ++a){
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
