#include "BaxterGBI_gui/menupanel.h"
#include "ui_menupanel.h"

#include <QDebug>

MenuPanel::MenuPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MenuPanel)
{
    ui->setupUi(this);
}

MenuPanel::~MenuPanel(){
    delete ui;
}
