#include "BaxterGBI_gui/actionpanel.h"
#include "ui_actionpanel.h"

#include <QDebug>

ActionPanel::ActionPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ActionPanel)
{
    ui->setupUi(this);
}

ActionPanel::~ActionPanel(){
    delete ui;
}
