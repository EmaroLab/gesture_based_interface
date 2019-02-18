/** @file BaxterDisplay.h
 * 
 *  @brief Functions and variables of a class that encapsulates pixmap 
 *  conversion and dispatches it to a Rethink Robotics Baxter LCD screen.
 *
 *  @author Lucrezia Grassi
 *  @author Patrick Roncagliolo
 */

#include <QString>
#include <QPixmap>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

/** @brief class which contains the main method and variables to make 
 *  possible to show the GUI on the Baxter's screen.
 */
class BaxterDisplay {
public:
		/** @brief contructor
		 * 
		 *  Creates a BaxterDisplay instance that will publish a message on 
		 *  the topic passed as parameter.
		 * 
		 *  @param[in] topic string of type QString
	   */
    explicit BaxterDisplay(QString topic);
    
    /** @brief contructor
     * 
     *   Creates a BaxterDisplay instance that will publish a message on 
		 *   the topic passed as parameter.
		 * 
		 *   @param[in] topic string of type std::string
	   */
    explicit BaxterDisplay(std::string topic);
    
    /** @brief overloading of the () operator which generates a png 
     *  image and sends it to the Baxter's screen.
     * 
     *  The pixmap object is converted to image in BGR_8888 format,
     *  then a ROS message containing the desired dimension and the format 
     *  of the image is generated and then published.
     * 
		 *  @param[in] pixmap off-screen image representation which can be
		 *  used as a paint device.
	   */
    void operator()(QPixmap &pixmap);
    
private:
    ros::NodeHandle n; /**< public node handle */
    ros::Publisher publisher; /**< publisher on topic passed as parameter*/
    sensor_msgs::Image message; /**< message published */
    std::string topic; /**< topic where the message must be published*/
};
