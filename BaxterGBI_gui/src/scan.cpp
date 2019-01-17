#include <string>
#include <map>
#include <vector>
#include <regex>
#include <initializer_list>
#include "ros/ros.h"
#include "ros/master.h"

/* test with 
rostopic pub /example/1 BaxterGBI_input_msgs/signal '{header: auto, device_id: "1", device_type: "smartwatch", device_model: "Huawei watch 2", action_descr: "wrist up", confidence: 1.0}' -r 1

*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_scanner");
  ros::NodeHandle n;

  ROS_INFO("Ready to scan.");


  ros::master::V_TopicInfo topicMap;
  std::map<std::string, std::vector<std::string>> compatibleSubtopics;
  std::regex regex("^\\/([a-zA-Z][0-9a-zA-Z_]+)\\/([0-9a-zA-Z_]+)$");
  std::smatch match;
  ros::master::getTopics(topicMap);

  for (ros::master::TopicInfo ti: topicMap){
    if (ti.datatype == "BaxterGBI_input_msgs/signal"){
      ROS_INFO("%s | %s", ti.name.c_str(), ti.datatype.c_str());
      if (std::regex_match(ti.name, match, regex)){
        static std::string topic, subtopic;
        topic = match[1];
        subtopic = match[2];
        ROS_INFO("topic: %s | subtopic: %s", topic.c_str(), subtopic.c_str());
        auto [__discard, first_sub] = compatibleSubtopics.try_emplace(topic, std::initializer_list<std::string>{subtopic});
        if (first_sub){
          ROS_INFO("This was the first subtopic of that topic");
        } else {
          compatibleSubtopics[topic].push_back(subtopic);
        }
      }
    }
  }

  for (auto v: compatibleSubtopics){
    ROS_INFO("Topic %s:", v.first.c_str());  
    for(auto s: v.second){
      ROS_INFO("\t%s", s.c_str());  
    }
  }

  return 0;
}
