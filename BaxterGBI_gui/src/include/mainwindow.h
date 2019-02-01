#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QString>
#include "ros/ros.h"

#include "configpanel.h"
#include "actionpanel.h"
#include "menupanel.h"
#include "rosworker.h"
#include "BaxterDisplay.h"
#include "BaxterGBI_core_msgs/status.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

protected:
  virtual void closeEvent(QCloseEvent *event) override;

private:
	Ui::MainWindow *ui;
	QThread *rosThread;
	Worker *worker;
	ConfigPanel conf_page;
	ActionPanel action_page;
	MenuPanel menu_page;
	QWidget *current_page;
	BaxterDisplay display;


private slots:
	void showConfig();
    void showMenu(QString &title,
                  QVector<QString> &options,
                  QVector<QString> &fixed_options,
                  int8_t selection);
    void showAction(QString action, 
                    QString msg);
    void switchPage(QWidget *target_page);
	void __setConfigMode();
	void __setActionMode();
	void __setMenuMode();
};

#endif // MAINWINDOW_H
