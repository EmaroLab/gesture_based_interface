#include "actionpanel.h"
#include "ui_actionpanel.h"
#include <QApplication>
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
  // the images are in the images folder and they are named as the action.
  if(actions.contains(action))
    logo.load(QString(":/images/%1.png").arg(action));
  //set image and message in the GUI
	ui->label->setPixmap(logo);
	ui->message->setText(msg);

  qApp->processEvents();
}
