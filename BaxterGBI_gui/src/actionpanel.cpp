#include "actionpanel.h"
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

void ActionPanel::update(QString action, QString msg){
  QPixmap logo;
  static QList<QString> actions{"play", "pause", "stop", "rec", "wait","config"};
  if(actions.contains(action))
    logo.load(QString(":/images/%1.png").arg(action));
	ui->label->setPixmap(logo);
	ui->message->setText(msg);
}
