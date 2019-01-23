#include "BaxterGBI_gui/menupanel.h"
#include "ui_menupanel.h"

#include <QDebug>
#include <QLabel>

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
}
