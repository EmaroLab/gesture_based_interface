/** @file TopicScanner.h
 *  @brief Function prototypes and variables for the Topic Scanner class
 * 
 *  @author Lucrezia Grassi
 */
#include "ros/ros.h"
#include "ros/master.h"
#include <QVector>
#include <QMap>
#include <QString>
#include <QRegularExpression>

/** @brief TopicScanner class which contains the main methods and variables 
 *  used to check if there are new topics available during the scan
 */
class TopicScanner{
public:
		/** @brief constructor
		 *  @param[in] datatype type of messages expected: "baxter_gbi_input_msgs/signal"
		 */
    explicit TopicScanner(QString datatype);
    
    /** @brief overloading of () operator
     * 
     *  This function resets the content of the tabs and if the names 
     *  of the topics match with the expected one the number of
     *  available topics is incremented
     * 
     */
    void operator()();
    
    QMap<QString, QVector<QString>>::iterator begin();
    QMap<QString, QVector<QString>>::iterator end();
    
    /** @brief returns the number of available topics */
    int count();
    
private:
    ros::master::V_TopicInfo topicMap; /**< map of the topics */
    QRegularExpression regex;/**< regular expression to be matched */
    QMap<QString, QVector<QString>> topics; /**< map with topics as key */
    QString datatype; /**< type of messages expected: "baxter_gbi_input_msgs/signal"*/
    int n_topics; /**<number of topics that can be added */
};
