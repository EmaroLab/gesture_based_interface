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
	void updateActionPanel(std::string pbr_action, std::string pbr_msg);
	~ActionPanel();

private:
	Ui::ActionPanel* ui;

private slots:

signals:
 
};

#endif // ACTIONPANEL_H
