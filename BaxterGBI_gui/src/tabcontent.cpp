#include "BaxterGBI_gui/tabcontent.h"
#include "ui_tabcontent.h"

#include "BaxterGBI_gui/mapping.h"

#include <QPushButton>
#include <QDebug>

TabContent::TabContent(QStandardItemModel *model, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TabContent),
    model(model)
{
    ui->setupUi(this);
    ui->addMappingButton->setEnabled(false); //disable button at the beginning
    connect(ui->addMappingButton, &QPushButton::clicked, this, &TabContent::addMapping);
}

TabContent::~TabContent(){
    delete ui;
}

void TabContent::addMapping(){
    Mapping* mapping = new Mapping(model);
    ui->topicsContainer->addWidget(mapping);
    connect(mapping, &Mapping::removed, this, &TabContent::removeMapping);
}

QVector<QPair<QString, QString>> TabContent::getTopics(){
    return {};
}

void TabContent::removeMapping(Mapping* mapping){
    ui->topicsContainer->removeWidget(mapping);
    mapping->setParent(nullptr);
    delete mapping;
}

void TabContent::clear(){
	qInfo() << "aaa";
	qDeleteAll(ui->topicsContainer->children());
}

void TabContent::enableAddButton(){
	ui->addMappingButton->setEnabled(true);
}
