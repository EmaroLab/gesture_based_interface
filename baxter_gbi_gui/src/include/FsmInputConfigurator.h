#include <string>
#include "ros/ros.h"
#include "TriggerService.h"
#include <QString>
#include <QVector>
#include <QPair>

class FsmInputConfigurator {
public:
    explicit FsmInputConfigurator(QString prefix, QString suffix);
    explicit FsmInputConfigurator(QString pattern);
    void operator()(int key, QVector<QPair<QString, QString>> topics) const;
    void operator()(int key, QVector<QString> topics) const;
    void commit();
private:
    ros::NodeHandle n;
    QString mask;
    TriggerService fsmReconfigure;
};
