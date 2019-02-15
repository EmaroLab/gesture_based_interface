#include "FsmInputConfigurator.h"
#include <string>
#include <vector>
#include "general_utilities.h"

FsmInputConfigurator::FsmInputConfigurator(QString prefix, QString suffix)
: FsmInputConfigurator(prefix+"%1"+suffix){}

FsmInputConfigurator::FsmInputConfigurator(QString mask)
: mask(mask)
, fsmReconfigure("/fsm_config"){}

void FsmInputConfigurator::operator()(int key, QVector<QPair<QString, QString>> topics) const {
    QVector<QString> serializedTopics;
    for(auto topic : topics)
      serializedTopics.append(QString("/%1/%2").arg(topic.first).arg(topic.second));
    (*this)(key, serializedTopics);
}

void FsmInputConfigurator::operator()(int key, QVector<QString> topics) const {
    n.setParam(mask.arg(key).toStdString(), qt2cxx_strvec(topics));
}

void FsmInputConfigurator::commit(){
    fsmReconfigure();
}
