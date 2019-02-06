#include <QString>
#include <string>
#include "ros/ros.h"
#include "baxter_gbi_input_msgs/signal.h"

class KeystrokePublisher {
public:
    explicit KeystrokePublisher(QString topic);
    void operator()();
private:
    ros::NodeHandle n;
    ros::Publisher publisher;
    baxter_gbi_input_msgs::signal msg;
    std::string topic;
};
