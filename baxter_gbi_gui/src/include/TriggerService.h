#include <string>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"

class TriggerService {
public:
    explicit TriggerService(std::string srv_name);
    void operator()();
private:
    ros::NodeHandle n;
    ros::ServiceClient client;
    std_srvs::Trigger request;
    std::string srv_name;
};
