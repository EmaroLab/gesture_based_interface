#include "tabcontent.h"
#include "mapping.h"
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
	emit numberOfMappings(count);
}

void TabContent::removeMapping(Mapping* mapping){
	ui->topicsContainer->removeWidget(mapping);
    //mapping->setParent(nullptr);
    mappings.removeOne(mapping);
    count--;
    delete mapping;
    emit numberOfMappings(count);
    emit mappingRemoved();
}

void TabContent::clear(){
    while(not mappings.isEmpty())
        removeMapping(mappings[0]);
}
