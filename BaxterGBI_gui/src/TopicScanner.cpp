#include "TopicScanner.h"

TopicScanner::TopicScanner(QString datatype)
: regex("^\\/([a-zA-Z][0-9a-zA-Z_]*)\\/([0-9a-zA-Z_]+)$")
, datatype(datatype)
, n_topics(0)
{}

void TopicScanner::operator()(){
  n_topics = 0;
  topicMap.clear();
  topics.clear();
  ros::master::getTopics(topicMap);
    
  for (auto ti: topicMap){
    if (QString::fromStdString(ti.datatype) == datatype){
      QString name = QString::fromStdString(ti.name);
      auto match = regex.match(name);
      if (match.hasMatch()){
        ++n_topics;
        auto topic = match.captured(1);
        auto subtopic = match.captured(2);
        auto iterator = topics.find(topic);
        if (iterator != topics.end()){
            iterator.value().append(subtopic);
        } else {
            topics.insert(topic, QVector<QString>{subtopic});
        }
      }
    }
  }
}

QMap<QString, QVector<QString>>::iterator TopicScanner::begin(){
    return topics.begin();
}

QMap<QString, QVector<QString>>::iterator TopicScanner::end(){
    return topics.end();
}

int TopicScanner::count(){
    return n_topics;
}
