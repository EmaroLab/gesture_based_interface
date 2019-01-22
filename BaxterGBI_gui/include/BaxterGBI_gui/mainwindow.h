#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "BaxterGBI_gui/configpanel.h"
#include "BaxterGBI_gui/rosworker.h"
#include "BaxterGBI_core_msgs/status.h"

#include "ros/ros.h"

#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QString>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
     void closeEvent(QCloseEvent *event) override;

private:
    Ui::MainWindow *ui;
    QThread *rosThread;
    Worker *worker;
    ConfigPanel conf_page;
    QWidget *current_page;

private slots:
    void updatePage(const boost::shared_ptr<BaxterGBI_core_msgs::status> msg);
};

#endif // MAINWINDOW_H
