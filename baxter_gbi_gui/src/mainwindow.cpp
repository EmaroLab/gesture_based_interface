#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "ros/ros.h"

#include <stdexcept>
#include <QCloseEvent>
#include <QPushButton>
#include <QImage>
#include <QKeyEvent>

#define SHARED_KEY_PUB(x) QSharedPointer<KeystrokePublisher>(new KeystrokePublisher("/keyboard/key_" #x))
#define SHARED_KEY_PUB_MAPPING(x) {Qt::Key_##x, SHARED_KEY_PUB(x)}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , rosThread(new QThread)
    , worker(new Worker)
    , current_page(nullptr)
    , display(QString("/robot/xdisplay"))
    , map{
          SHARED_KEY_PUB_MAPPING(1),
          SHARED_KEY_PUB_MAPPING(2),
          SHARED_KEY_PUB_MAPPING(3),
          SHARED_KEY_PUB_MAPPING(4),
          SHARED_KEY_PUB_MAPPING(5),
          SHARED_KEY_PUB_MAPPING(6),
          SHARED_KEY_PUB_MAPPING(7),
          SHARED_KEY_PUB_MAPPING(8),
          SHARED_KEY_PUB_MAPPING(9),
          SHARED_KEY_PUB_MAPPING(0),
          SHARED_KEY_PUB_MAPPING(Up),
          SHARED_KEY_PUB_MAPPING(Down),
          SHARED_KEY_PUB_MAPPING(Left),
          SHARED_KEY_PUB_MAPPING(Right),
          SHARED_KEY_PUB_MAPPING(Super_L),
          SHARED_KEY_PUB_MAPPING(Super_R),
          SHARED_KEY_PUB_MAPPING(PageUp),
          SHARED_KEY_PUB_MAPPING(PageDown),
          SHARED_KEY_PUB_MAPPING(End),
          SHARED_KEY_PUB_MAPPING(Control),
          SHARED_KEY_PUB_MAPPING(Shift)
      }
{
	ui->setupUi(this);
	worker->moveToThread(rosThread);
	connect(rosThread, &QThread::started, worker, &Worker::start);
	connect(worker, &Worker::finished, [&] {rosThread->quit();});
	connect(worker, &Worker::finished, worker, &Worker::deleteLater);
	connect(rosThread, &QThread::finished, rosThread, &QThread::deleteLater);

	connect(worker, &Worker::configFrame, this, &MainWindow::showConfig);
	connect(worker, &Worker::menuFrame, this, &MainWindow::showMenu);
	connect(worker, &Worker::actionFrame, this, &MainWindow::showAction);

	rosThread->start();
}

MainWindow::~MainWindow(){
	delete ui;
}

void MainWindow::showConfig(){
	action_page.update("config", "Waiting for configuration");
	switchPage(&conf_page);
	auto pixmap = action_page.grab();
    display(pixmap);
}

void MainWindow::showMenu(QString title,
                          QVector<QString> options,
                          QVector<QString> fixed_options,
                          char selection){
	menu_page.update(title, options, fixed_options, selection);
	switchPage(&menu_page);
	auto pixmap = menu_page.grab();
    display(pixmap);
}

void MainWindow::showAction(QString action, QString msg){
	action_page.update(action, msg);
	switchPage(&action_page);
	auto pixmap = action_page.grab();
    display(pixmap);
}

void MainWindow::switchPage(QWidget *target_page){
	if (current_page != target_page){
		if (current_page) {
			ui->innerLayout->removeWidget(current_page);
			current_page->setParent(nullptr);
		}
		if (target_page){
			ui->innerLayout->addWidget(target_page);
		}
	}
	current_page = target_page;
}

void MainWindow::closeEvent(QCloseEvent *event){
	worker->stop();
	rosThread->wait();
	event->accept();
}

void MainWindow::__setConfigMode(){
    showConfig();
}

void MainWindow::__setActionMode(){
    switchPage(&action_page);
    auto pixmap = action_page.grab();
    display(pixmap);
}

void MainWindow::__setMenuMode(){
    switchPage(&menu_page);
    auto pixmap = menu_page.grab();
    display(pixmap);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event){
		auto key = map.value(event->key(), nullptr);
		if(key) (*key)();
}
