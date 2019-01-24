#include "BaxterGBI_gui/actionpanel.h"
#include "ui_actionpanel.h"

#include <QDebug>
#include <QLabel>
#include <QPixmap>
#include <QVBoxLayout>
#include <QBitmap>
#include <QSize>
#include <QWidget>

ActionPanel::ActionPanel(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::ActionPanel)
{
	ui->setupUi(this);
}

ActionPanel::~ActionPanel(){
	delete ui;
}

void ActionPanel::updateActionPanel(std::string pbr_action, std::string pbr_msg){
	QString action = QString::fromStdString(pbr_action);
	QString msg = QString::fromStdString(pbr_msg);
	
	QPixmap logo;
	
	if(action == "play"){
		logo.load("play.png");
		ui->label->setPixmap(logo);
	}
	else if(action == "pause"){
		logo.load("pause.png");
		ui->label->setPixmap(logo);
	}
	else if(action == "stop"){
		logo.load("stop.png");
		ui->label->setPixmap(logo);
	}
	else if(action == "rec"){
		logo.load("rec.png");
		ui->label->setPixmap(logo);
	}
	else if(action == "wait"){
		logo.load("wait.png");
		ui->label->setPixmap(logo);
	}
	
	ui->message->setStyleSheet("QLabel { font: 15pt Comic Sans MS;"
																"font-style:italic;"
																"padding:5px;}");
	ui->message->setText(msg);
	
}
