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

private slots:

signals:
 
};

#endif // MENUPANEL_H
