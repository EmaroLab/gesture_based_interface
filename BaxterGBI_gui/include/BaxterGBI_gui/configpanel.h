#ifndef CONFIGPANEL_H
#define CONFIGPANEL_H

#include <QMainWindow> //our MainWindow object inherits from QMainWindow class
#include <QVector> //Qt container class providing a dynamic array (e.g std::vector)

#include "mapping.h"

namespace Ui {
class ConfigPanel;
}

class ConfigPanel : public QMainWindow
{
    Q_OBJECT //allows the class to define its own signals/slots

public:
    explicit ConfigPanel(QWidget* parent = nullptr); //QWidget is a UI component
    ~ConfigPanel();

private:
    Ui::ConfigPanel* ui; //forward declaration

private slots:
    void scan();
};

#endif // CONFIGPANEL_H
