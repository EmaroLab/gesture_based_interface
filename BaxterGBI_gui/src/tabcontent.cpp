#include "BaxterGBI_gui/tabcontent.h"
#include "ui_tabcontent.h"

#include "BaxterGBI_gui/mapping.h"

#include <QPushButton>

TabContent::TabContent(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TabContent)
{
    ui->setupUi(this);
    connect(ui->addMappingButton, &QPushButton::clicked, this, &TabContent::addMapping);
}

TabContent::~TabContent()
{
    delete ui;
}

void TabContent::addMapping()
{
    Mapping* mapping = new Mapping();
    ui->topicsContainer->addWidget(mapping);
    connect(mapping, &Mapping::removed, this, &TabContent::removeMapping);
}

QVector<QPair<QString, QString>> TabContent::getTopics(){
    return {};
}

void TabContent::removeMapping(Mapping* mapping)
{
    ui->topicsContainer->removeWidget(mapping);
    mapping->setParent(nullptr);
    delete mapping;
}

