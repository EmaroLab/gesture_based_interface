#ifndef MENUPANEL_H
#define MENUPANEL_H

#include <vector>
#include <string>
#include <QWidget>
#include <QPushButton>
#include <QVector>

namespace Ui {
class MenuPanel;
}

class MenuPanel : public QWidget
{
	Q_OBJECT

public:
	explicit MenuPanel(QWidget* parent = nullptr);
	void updateMenuPanel(std::string m_title, std::vector<std::string> m_options,
											 std::vector<std::string> m_fixed_options, int8_t m_selection);
	~MenuPanel();

private:
	Ui::MenuPanel* ui;
	QVector<QPushButton*> optionsButtons;
	QVector<QPushButton*> fixedOptionsButtons;
	//style
	QString titleStyle = "QLabel { font: 20pt Comic Sans MS;"
																"font-style:bold;}";
	QString style = "QPushButton{"
										"font: 15pt Comic Sans MS;"
										"background-color: white;"
										"height: 50px;"
										"border-radius:10px;"
										"border: none;"
										"padding: 6px;}";
	QString selectedStyle = "QPushButton{"
													"font: 15pt Comic Sans MS;"
													"background-color:#99ffff;"
													"border-style:none;"
													"border-radius: 10px;"
													"height: 50px;"
													"padding: 6px;}";

private slots:

signals:
 
};

#endif // MENUPANEL_H
