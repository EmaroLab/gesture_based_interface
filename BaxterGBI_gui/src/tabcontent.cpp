#include "BaxterGBI_gui/tabcontent.h"
#include "BaxterGBI_gui/mapping.h"
#include "ui_tabcontent.h"

#include <QPushButton>
#include <QDebug>

TabContent::TabContent(QStandardItemModel *model, QWidget *parent) :
	QWidget(parent),
	ui(new Ui::TabContent),
	model(model)
{
	ui->setupUi(this);
	count = 0;
	ui->addMappingButton->setEnabled(false); //disable add button at the beginning
	connect(ui->addMappingButton, &QPushButton::clicked, this, &TabContent::addMapping);
}

TabContent::~TabContent(){
	delete ui;
}

QVector<QPair<QString, QString>> TabContent::getSelectedTopics(){
	QVector<QPair<QString, QString>> selectedTopics;
	for(auto item : mappings)
		selectedTopics.append(item->currentSelection());
	
	return selectedTopics;
}

void TabContent::addMapping(){
	Mapping *mapping = new Mapping(model);
	mappings.append(mapping);
	ui->topicsContainer->addWidget(mapping);
	connect(mapping, &Mapping::removed, this, &TabContent::removeMapping);
	count++;
	qInfo() << count << "added";
	emit numberOfMappings(count);
}

void TabContent::removeMapping(Mapping* mapping){
	ui->topicsContainer->removeWidget(mapping);
	mapping->setParent(nullptr);
	mappings.removeOne(mapping);
	delete mapping;
	count--;
	//qInfo() << count << "deleted";
	emit numberOfMappings(count);
}

void TabContent::clear(){
	QLayoutItem* item;
	for(auto item : mappings){
		ui->topicsContainer->removeWidget(item);
		delete item;
	}
	mappings.clear();
  count = 0;
  emit numberOfMappings(count);
}

void TabContent::enableAddButton(bool enable){
	ui->addMappingButton->setEnabled(enable);
}
