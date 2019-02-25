#include "configpanel.h"
#include "ui_configpanel.h"

#include "ros/ros.h"
#include "ros/master.h"

#include <QDebug>
#include <QInputDialog>

ConfigPanel::ConfigPanel(QWidget *parent)
: QWidget(parent)
, ui(new Ui::ConfigPanel)
, model(new QStandardItemModel)
, isFilled(6, false)
, scanner("baxter_gbi_input_msgs/signal")
, fsmInputConfigurator("key_", "_topics")
{
	ui->setupUi(this);
	ui->loadConfigButton->setEnabled(false);
	ui->addMappingButton->setEnabled(false);
	ui->tabWidget->clear();
  
  // creation of 6 tabs
	for (int i = 0; i < 6; i++){
		tabs[i] = new TabContent(model);
		ui->tabWidget->addTab(tabs[i], QString("Action %1").arg(i+1));
		connect(tabs[i], &TabContent::numberOfMappings, 
                        [=](const int &mappings){enableLoadButton(i, mappings);
						});
		// when a mapping is removed from a tab the number of available
		// mapping is incremented
		connect(tabs[i], &TabContent::mappingRemoved,
										    [&](){++n_topics;
													ui->addMappingButton->setEnabled(n_topics);
						});
	}
	// connect slots to the trigger event
	connect(ui->scanButton, &QPushButton::clicked, this, &ConfigPanel::scan);
	connect(ui->addMappingButton, &QPushButton::clicked, this, &ConfigPanel::addMappingToActiveTab);
	connect(ui->loadConfigButton, &QPushButton::clicked, this, &ConfigPanel::sendConfig);
}

ConfigPanel::~ConfigPanel(){
	delete ui;
}

void ConfigPanel::scan(){
  for(int i = 0; i < 6; i++){
    tabs[i]->clear();
	}
  ui->addMappingButton->setEnabled(false);
  model->clear();
  scanner();
  if(scanner.count() < 6) return;
  n_topics = scanner.count();

  for(auto a = scanner.begin(); a != scanner.end(); ++a){
    qInfo() << "Topic " << a.key() << ": ";
    ROS_INFO("Topic %s:", a.key().toLatin1().data());
    auto topic = new QStandardItem(a.key());
    model->appendRow(topic);
    for(auto b: a.value()){
      qInfo() << "\t" << b;
      ROS_INFO("\t%s", b.toLatin1().data());
      topic->appendRow(new QStandardItem(b));
    }
  }
  ui->addMappingButton->setEnabled(true);
}

void ConfigPanel::addMappingToActiveTab(){
  tabs[ui->tabWidget->currentIndex()]->addMapping();
  --n_topics;
  ui->addMappingButton->setEnabled(n_topics);
}

void ConfigPanel::enableLoadButton(int tab, int mappings){
  isFilled[tab] = mappings > 0;
	
  bool ok = true;
  for(int i = 0; i < 6 and ok; i++)
    ok = isFilled[i];
	
  ui->loadConfigButton->setEnabled(ok);
}

void ConfigPanel::sendConfig(){
  for(int i = 0; i < 6; i++)
    fsmInputConfigurator(i+1, tabs[i]->getSelectedTopics());

  fsmInputConfigurator.commit();
}
