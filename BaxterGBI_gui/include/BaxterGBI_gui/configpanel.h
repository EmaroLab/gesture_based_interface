#ifndef CONFIGPANEL_H
#define CONFIGPANEL_H

#include <QWidget>
#include <QVector>

#include "mapping.h"

namespace Ui {
class ConfigPanel;
}

class ConfigPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ConfigPanel(QWidget* parent = nullptr);
    ~ConfigPanel();

private:
    Ui::ConfigPanel* ui;

private slots:
    void scan();
};

#endif // CONFIGPANEL_H
