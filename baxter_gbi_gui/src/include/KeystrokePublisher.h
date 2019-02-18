/** @file KeystrokePublisher.h 
 *  @brief Functions and variables for the KeystrokePublisher class
 * 
 *  @author Lucrezia Grassi
 *  @author Patrick Roncagliolo
 */
#include <QString>
#include <string>
#include "ros/ros.h"
#include "baxter_gbi_input_msgs/signal.h"

/** @brief class which generates and publishes a message on a specific topic
 */
class KeystrokePublisher {
public:
    /** @brief constructor
     * 
     *  Creates a publisher which publishes on the topic passed as parameter
     *  @param[in] topic topic on which the publisher must publish the message
     */
    explicit KeystrokePublisher(QString topic);
    
    /** @brief overloading of () operator 
     * 
     *  When this function is called, the fields of the message are filled
     *  and the message is published.
     */
    void operator()();
private:
    ros::NodeHandle n; /**< public node handle*/
    ros::Publisher publisher; /**< publisher */
    baxter_gbi_input_msgs::signal msg; /**< message defined by us */
    std::string topic; /**< topic on which the publisher must publish */
};
