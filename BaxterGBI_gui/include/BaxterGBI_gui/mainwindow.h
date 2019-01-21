#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow> //our MainWindow object inherits from QMainWindow class
#include <QVector> //Qt container class providing a dynamic array (e.g std::vector)

#include "mapping.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT //allows the class to define its own signals/slots

public:
    explicit MainWindow(QWidget* parent = nullptr); //QWidget is a UI component
    ~MainWindow();

private:
    Ui::MainWindow* ui; //forward declaration

private slots:
    void scan();
};

#endif // MAINWINDOW_H
