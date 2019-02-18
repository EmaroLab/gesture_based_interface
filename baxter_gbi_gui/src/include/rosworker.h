/** @file rosworker.h 
 *  @brief Functions and variables for the Worker object
 *  
 *  @author Lucrezia Grassi 
 *  @author Patrick Roncagliolo 
 */
#ifndef ROSWORKER_H
#define ROSWORKER_H

#include "baxter_gbi_core_msgs/status.h"
#include "ros/ros.h"

#include <QObject>
#include <QVector>
#include <QString>

/** @brief class which subscribes to the "/fsm_status" topic and acts as
 *  interface between ROS and the Qt framework.
 *  It's advisable to run this object in a background thread otherwise 
 *  the menu is blocked.
 */
class Worker : public QObject{
	Q_OBJECT
    
private:
	ros::Subscriber sub; /**< ros subscriber */
	ros::NodeHandle n; /**< node handle */
    
public:
  /** @brief constructor
   */
	explicit Worker(QWidget *parent = 0);
	
	/** @brief destructor
	 */
	~Worker();

public slots:
  /** @brief subscribes to the topic "/fsm_status"
   */
	void start();
	
	/** @brief shutdown
	 */
	void stop();
    
signals:
	/** @brief 
	 */
	void configFrame();

	/** @brief emitted when the field context_type of the message received
	 *  is "menu".
	 * 
	 *  @param[in] title menu page title
	 *  @param[in] options options of the menu in the scroll area
	 *  @param[in] fixed_options menu fixed options in the bottom area
	 *  @param[in] selection selected element of the menu
	 */
	void menuFrame(QString title,
		       QVector<QString> options,
		       QVector<QString> fixed_options,
		       char selection);
	
	/** @brief emitted when the field context_type of the message received
	 *  is "action".
	 *  
	 *  @param[in] action current state of the robot. 
   *  According to this parameter the GUI image will be updated.
   *  @param[in] msg message associated with the current state shown 
   *  the image.
	 */
	void actionFrame(QString action, 
			 QString msg);

	/** @brief emitted after the message on the topic is read
	 */
	void finished();

private:
  /** @brief checks the context_type field of the message received 
   *  and updates the GUI with the right panel.
   *  
   *  @param[in] msg message received
   */
	void statusCb(const boost::shared_ptr<baxter_gbi_core_msgs::status> msg);
};

#endif //ROSWORKER_H
