#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <stdexcept>
#include <QCloseEvent>
#include <QPushButton>
#include <QImage>

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow),
	rosThread(new QThread),
	worker(new Worker),
	current_page(nullptr),
	display(QString("/robot/xdisplay"))
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

	connect(ui->configModeButton, &QPushButton::clicked, this, &MainWindow::__setConfigMode);
	connect(ui->actionModeButton, &QPushButton::clicked, this, &MainWindow::__setActionMode);
	connect(ui->menuModeButton, &QPushButton::clicked, this, &MainWindow::__setMenuMode);
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

void MainWindow::showMenu(QString &title,
                          QVector<QString> &options,
                          QVector<QString> &fixed_options,
                          int8_t selection){
	menu_page.update(title, options, fixed_options, selection);
	switchPage(&menu_page);
	auto pixmap = menu_page.grab();
    display(pixmap);
}

void MainWindow::showAction(QString action, 
                 QString msg){
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
    QVector<QString> a{"Test 1", "Test 2"};
    QString t("title");
    showMenu(t, a, a, 0);
}

void MainWindow::__setMenuMode(){
    showAction("play", "Example text");
}
