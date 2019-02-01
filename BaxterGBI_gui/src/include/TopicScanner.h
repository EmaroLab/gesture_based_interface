#include "ros/ros.h"
#include "ros/master.h"
#include <QVector>
#include <QMap>
#include <QString>
#include <QRegularExpression>

class TopicScanner {
public:
    explicit TopicScanner(QString datatype);
    void operator()();
    QMap<QString, QVector<QString>>::iterator begin();
    QMap<QString, QVector<QString>>::iterator end();
    int count();
private:
    ros::master::V_TopicInfo topicMap;
    QRegularExpression regex;
    QMap<QString, QVector<QString>> topics;
    QString datatype;
    int n_topics;
};
