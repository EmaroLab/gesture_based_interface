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

QVector<QPair<QString, QString>> TabContent::getTopics(){
    return {};
}

void TabContent::addMapping(){
		
    Mapping* mapping = new Mapping(model);
    ui->topicsContainer->addWidget(mapping);
    connect(mapping, &Mapping::removed, this, &TabContent::removeMapping);
    count++;
    qInfo() << count << "added";
    emit numberOfMappings(count);
}

void TabContent::removeMapping(Mapping* mapping){
    ui->topicsContainer->removeWidget(mapping);
    mapping->setParent(nullptr);
    delete mapping;
    count--;
    //qInfo() << count << "deleted";
    emit numberOfMappings(count);
}

void TabContent::clear(){
	QLayoutItem* item;
	while ((item = ui->topicsContainer->layout()->takeAt(0)) != nullptr){
		delete item->widget();
		delete item;
	}
  count = 0;
  emit numberOfMappings(count);
}

void TabContent::enableAddButton(bool enable){
	ui->addMappingButton->setEnabled(enable);
}
