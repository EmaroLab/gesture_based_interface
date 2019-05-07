#include "menupanel.h"
#include "ui_menupanel.h"
#include <QApplication>
#include <QDebug>
#include <QLabel>
#include <QVector>

#include "general_utilities.h"

MenuPanel::MenuPanel(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::MenuPanel)
{
	ui->setupUi(this);
}

MenuPanel::~MenuPanel(){
	delete ui;
}

void MenuPanel::update(QString &title,
                       QVector<QString> &options,
                       QVector<QString> &fixed_options,
                       int8_t selection){
	
	ui->menuTitle->setText(title);

	//delete previous items in the menu, if present
  clearContainer(ui->optionsContainer);
  clearContainer(ui->fixedOptionsContainer);

  optionsButtons.clear();

  //add options buttons to scroll area
  for (auto option: options){
      auto button = new QPushButton(option, ui->scrollArea->widget());
      optionsButtons.push_back(button);
      ui->optionsContainer->addWidget(button);
  }

	//vertical spacer
	ui->optionsContainer->addStretch();
	
	//add fixed options buttons to bottom area
  for (auto option: fixed_options){
      auto button = new QPushButton(option, this);
      optionsButtons.push_back(button);
      ui->fixedOptionsContainer->addWidget(button);
  }

	//manage selection
  optionsButtons.at(selection % optionsButtons.size())
                ->setObjectName("selection");
  qApp->processEvents();
  ui->scrollArea->ensureWidgetVisible(optionsButtons.at(selection),5,5);
}
