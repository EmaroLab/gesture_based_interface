#include "BaxterGBI_gui/mapping.h"
#include "ui_mapping.h"

#include <QInputDialog>
#include <QDebug>
#include <QComboBox>

Mapping::Mapping(QStandardItemModel *model, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Mapping),
    model(model)
{
    ui->setupUi(this);
    connect(ui->deleteMapping, &QPushButton::clicked, [this] {
        emit removed(this);
    });
   
    ui->topic->setModel(model);
   	ui->subtopic->setModel(model);
   	updateSubtopics(0);
    connect(ui->topic, 
			static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), 
			this, 
			&Mapping::updateSubtopics);
}

void Mapping::updateSubtopics(int idx){
	if (idx != -1){
		ui->subtopic->setRootModelIndex(model->item(idx,0)->index());
		ui->subtopic->setCurrentIndex(0);
	}
}

Mapping::~Mapping(){
    delete ui;
}


