#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QKeyEvent>
#include <QString>
#include <QMap>
#include <QSharedPointer>
#include "ros/ros.h"

#include "configpanel.h"
#include "actionpanel.h"
#include "menupanel.h"
#include "rosworker.h"
#include "BaxterDisplay.h"
#include "KeystrokePublisher.h"

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
  QMap<int, QSharedPointer<KeystrokePublisher>> map;


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
  void keyReleaseEvent(QKeyEvent *event);
};

#endif // MAINWINDOW_H
