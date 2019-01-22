#ifndef CONFIGPANEL_H
#define CONFIGPANEL_H

#include <QWidget>
#include <QVector>
#include <QStandardItemModel>

#include "mapping.h"
#include <map>
#include <vector>
#include <string>

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
    QStandardItemModel* model;
    std::map<std::string, std::vector<std::string>> compatibleSubtopics;

private slots:
    void scan();

signals:
    void scan_terminated();
};

#endif // CONFIGPANEL_H
