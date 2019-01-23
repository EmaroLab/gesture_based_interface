#ifndef CONFIGPANEL_H
#define CONFIGPANEL_H

#include "mapping.h"
#include <map>
#include <vector>
#include <string>

#include <QWidget>
#include <QVector>
#include <QStandardItemModel>

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
    QVector<bool> isFilled;
    bool *data;
    int mappings;
    int filledTabs;

private slots:
    void scan();
    void enableLoadButton(int tab, int count);

signals:
    void scan_terminated();
};

#endif // CONFIGPANEL_H
