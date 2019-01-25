#include "BaxterGBI_gui/menupanel.h"
#include "ui_menupanel.h"

#include <QDebug>
#include <QLabel>
#include <QVector>

MenuPanel::MenuPanel(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::MenuPanel)
{
	ui->setupUi(this);
}

MenuPanel::~MenuPanel(){
	delete ui;
}

void MenuPanel::updateMenuPanel(std::string m_title, std::vector<std::string> m_options,
																std::vector<std::string> m_fixed_options, int8_t m_selection){
	
	QString title = QString::fromStdString(m_title);
	ui->menuTitle->setText(title);

	//delete previous items in the menu, if present
	QLayoutItem* item;
	while ((item = ui->optionsContainer->layout()->takeAt(0)) != nullptr){
		delete item->widget();
    delete item;
	}
  while((item = ui->fixedOptionsContainer->layout()->takeAt(0)) != nullptr){
		delete item->widget();
    delete item;
	}
	optionsButtons.clear();
	fixedOptionsButtons.clear();
	
	//add options buttons to scroll area
	QVector<QString> options;
	for(auto option : m_options){
		auto text = QString::fromStdString(option);
		options.append(text);
		auto button = new QPushButton(text);
		optionsButtons.push_back(button);
		ui->optionsContainer->addWidget(button);
		//button->setAutoDefault(false);
		//button->setDefault(false);
	}
	
	//vertical spacer
	ui->optionsContainer->addStretch();
	
	//add fixed options buttons to bottom area
	QVector<QString> fixedOptions;
	for(auto fixedOption : m_fixed_options){
		auto text = QString::fromStdString(fixedOption);
		fixedOptions.append(text);
		auto button = new QPushButton(text);
		fixedOptionsButtons.push_back(button);
		ui->fixedOptionsContainer->addWidget(button);		
	}

	//manage selection
	m_selection = m_selection % (optionsButtons.size() + fixedOptionsButtons.size());
	if(m_selection >= 0 and m_selection < optionsButtons.size()){
		optionsButtons.at(m_selection)->setObjectName("selection");
	}
	else if (m_selection >= optionsButtons.size() and m_selection < (optionsButtons.size() + fixedOptionsButtons.size()))
		fixedOptionsButtons.at(m_selection - options.size())->setObjectName("selection");
}
