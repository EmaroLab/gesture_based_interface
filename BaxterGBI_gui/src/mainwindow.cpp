#include "BaxterGBI_gui/mainwindow.h"
#include "ui_mainwindow.h"
#include "sensor_msgs/Image.h"

#include <stdexcept>
#include <QCloseEvent>
#include <QPushButton>
#include <QImage>

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
	
  ros::NodeHandle n;
	display_pub = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1);
}

MainWindow::~MainWindow(){
	delete ui;
}

void MainWindow::updateMainWindow(const boost::shared_ptr<BaxterGBI_core_msgs::status> msg){
  static QWidget *target_page;
  QPixmap pixmap(ui->widget->size());
  if(msg->context_type == "config_wait" or msg->context_type == "wait_user"){
		target_page = &conf_page;
		action_page.update("config", "Waiting for configuration");
		pixmap = action_page.grab();
	} else if (msg->context_type == "menu"){
		target_page = &menu_page;
		auto title = QString::fromStdString(msg->m_title);
		auto options = cxx2qt_strvec(msg->m_options);
		auto fixed_options = cxx2qt_strvec(msg->m_fixed_options);
		menu_page.update(title, options, fixed_options, msg->m_selection);
		pixmap = menu_page.grab();
	} else if (msg->context_type == "action"){
	target_page = &action_page;
		auto action = QString::fromStdString(msg->pbr_action);
		auto message = QString::fromStdString(msg->pbr_msg);
		action_page.update(action, message);
		pixmap = action_page.grab();
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
	 
	auto img_rgb = pixmap.toImage().convertToFormat(QImage::Format_RGB888);
	auto img_bgr = img_rgb.rgbSwapped();
	//img_rgb.save("/home/lucrezia/file_rgb.png", "PNG", 100);
	//img_bgr.save("/home/lucrezia/file_bgr.png", "PNG", 100);
	
	sensor_msgs::Image out_msg;
	out_msg.height = 600;
	out_msg.width = 1024;
	out_msg.encoding = "bgr8";
	out_msg.is_bigendian = 0;
	out_msg.step = img_bgr.bytesPerLine();
	out_msg.data.assign(img_bgr.constBits(), img_bgr.constBits() + (1024*600*3));
	display_pub.publish(out_msg);
	ros::spinOnce();
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
