#ifndef MENUPANEL_H
#define MENUPANEL_H

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
  void update(QString &title,
              QVector<QString> &options,
              QVector<QString> &fixed_options,
              int8_t selection);
	~MenuPanel();

private:
	Ui::MenuPanel* ui;
	QVector<QPushButton*> optionsButtons; 
};

#endif // MENUPANEL_H
