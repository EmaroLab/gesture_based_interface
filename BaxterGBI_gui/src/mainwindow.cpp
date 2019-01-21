#include "BaxterGBI_gui/mainwindow.h"
#include "ui_mainwindow.h"
#include <QCloseEvent>

void MainWindow::updatePage(const boost::shared_ptr<BaxterGBI_core_msgs::status> msg){
    ROS_INFO("Context type: [%s]", msg->context_type.c_str());
    qInfo() << "Context type: " << msg->context_type.c_str();
    static QWidget *target_page;

    if (msg->context_type == "config_wait" or msg->context_type == "wait_user"){
        target_page = &conf_page;
    } else {
        target_page = nullptr;
    }

    if (current_page != target_page){
        if (current_page) {
            ui->innerLayout->removeWidget(current_page);
            current_page->setParent(nullptr);
        }
        if (target_page) {
            ui->innerLayout->addWidget(target_page);
        }
    }
    current_page = target_page;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    current_page(nullptr),
    rosThread(new QThread),
    worker(new Worker)
{
    ui->setupUi(this);
    worker->moveToThread(rosThread);
    connect(rosThread, &QThread::started, worker, &Worker::process);
    connect(worker, &Worker::finished, [&] {rosThread->quit();});
    connect(worker, &Worker::finished, worker, &Worker::deleteLater);
    connect(rosThread, &QThread::finished, rosThread, &QThread::deleteLater);
    connect(worker, &Worker::newStatus, this, &MainWindow::updatePage);
    rosThread->start();
}

void MainWindow::closeEvent(QCloseEvent *event){
    worker->stop();
    rosThread->wait();
    event->accept();
}

MainWindow::~MainWindow(){
    delete ui;
}
