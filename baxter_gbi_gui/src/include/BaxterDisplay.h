#include <QString>
#include <QPixmap>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

class BaxterDisplay {
public:
    explicit BaxterDisplay(QString topic);
    explicit BaxterDisplay(std::string topic);
    void operator()(QPixmap &pixmap);
private:
    ros::NodeHandle n;
    ros::Publisher publisher;
    sensor_msgs::Image message;
    std::string topic;
};
