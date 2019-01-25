#ifndef ACTIONPANEL_H
#define ACTIONPANEL_H

#include <string>
#include <QWidget>

namespace Ui {
class ActionPanel;
}

class ActionPanel : public QWidget
{
	Q_OBJECT

public:
	explicit ActionPanel(QWidget* parent = nullptr);
  void update(QString action, QString msg);
	~ActionPanel();

private:
	Ui::ActionPanel* ui;
};

#endif // ACTIONPANEL_H
