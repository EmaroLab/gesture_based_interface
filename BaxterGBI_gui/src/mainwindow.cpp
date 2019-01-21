#include "BaxterGBI_gui/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    w(new ConfigPanel)
{
    ui->setupUi(this);
    connect(ui->configButton, &QPushButton::clicked, &w, &QMainWindow::show);

}

MainWindow::~MainWindow()
{
    delete ui;
}
