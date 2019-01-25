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
		logo.load(":/images/play.png");	
	} else if(action == "pause"){
		logo.load(":/images/pause.png");
	} else if(action == "stop"){
		logo.load(":/images/stop.png");
	} else if(action == "rec"){
		logo.load(":/images/rec.png");
	} else if(action == "wait"){
		logo.load(":/images/wait.png");
	}
	ui->label->setPixmap(logo);
	ui->message->setText(msg);
}
