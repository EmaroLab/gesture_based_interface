#include "BaxterGBI_gui/mainwindow.h"
#include "ui_mainwindow.h"

#include <stdexcept>
#include <QCloseEvent>
#include <QPushButton>

QVector<QString> cxx2qt_strvec(std::vector<std::string> &stdvstds){
  std::vector<QString> v;
  v.clear();
  v.reserve(stdvstds.size());
  std::transform(stdvstds.begin(),
                 stdvstds.end(),
                 std::back_inserter(v),
                 QString::fromStdString);
  return QVector<QString>::fromStdVector(v);
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
	connect(worker, &Worker::newStatus, this, &MainWindow::updateMainWindow);
	rosThread->start();

	connect(ui->configModeButton, &QPushButton::clicked, this, &MainWindow::__setConfigMode);
	connect(ui->actionModeButton, &QPushButton::clicked, this, &MainWindow::__setActionMode);
	connect(ui->menuModeButton, &QPushButton::clicked, this, &MainWindow::__setMenuMode);
}

MainWindow::~MainWindow(){
	delete ui;
}

void MainWindow::updateMainWindow(const boost::shared_ptr<BaxterGBI_core_msgs::status> msg){
  static QWidget *target_page;

	if(msg->context_type == "config_wait" or msg->context_type == "wait_user"){
		target_page = &conf_page;
    } else if (msg->context_type == "menu"){
      target_page = &menu_page;
      auto title = QString::fromStdString(msg->m_title);
      auto options = cxx2qt_strvec(msg->m_options);
      auto fixed_options = cxx2qt_strvec(msg->m_fixed_options);
      menu_page.update(title, options, fixed_options, msg->m_selection);
    } else if (msg->context_type == "action"){
		target_page = &action_page;
      auto action = QString::fromStdString(msg->pbr_action);
      auto message = QString::fromStdString(msg->pbr_msg);
      action_page.update(action, message);
    } else {
      ROS_WARN("Wrong context type in received status msg.");
      return;
    };

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

void MainWindow::__setConfigMode(){
	if (current_page) {
		ui->innerLayout->removeWidget(current_page);
		current_page->setParent(nullptr);
	}
	ui->innerLayout->addWidget(&conf_page);
	current_page = &conf_page;
}

void MainWindow::__setActionMode(){
	if (current_page) {
		ui->innerLayout->removeWidget(current_page);
		current_page->setParent(nullptr);
	}
		ui->innerLayout->addWidget(&action_page);
		current_page = &action_page;
}

void MainWindow::__setMenuMode(){
	if (current_page) {
		ui->innerLayout->removeWidget(current_page);
		current_page->setParent(nullptr);
	}
	ui->innerLayout->addWidget(&menu_page);
	current_page = &menu_page;
}

void MainWindow::closeEvent(QCloseEvent *event){
	worker->stop();
	rosThread->wait();
	event->accept();
}
