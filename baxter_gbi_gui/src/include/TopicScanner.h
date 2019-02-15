/** @file TopicScanner.h
 *  @brief Function prototypes and variables for the Topic Scanner class
 * 
 *  @author Lucrezia Grassi
 *  @author Patrick Roncagliolo
 */
#include "ros/ros.h"
#include "ros/master.h"
#include <QVector>
#include <QMap>
#include <QString>
#include <QRegularExpression>

/** @brief This class contains the main methods and variables 
 *  used to check if there are new topics available during the scan
 */
class TopicScanner{
public:
		/** @brief constructor
		 * 
		 *  @param[in] datatype type of messages expected: "baxter_gbi_input_msgs/signal"
		 */
    explicit TopicScanner(QString datatype);
    
    /** @brief overloading of () operator
     * 
     *  This function clears the map and the array of topics.
     *  If the names of the topics found matches with the expected one,
     *  the number of available topics is incremented.
     */
    void operator()();
    
    /** @brief this function returns the iterator to the first element
     *  of the results map.
     * 
     *  @return iterator to the first element of the results map.
     */
    QMap<QString, QVector<QString>>::iterator begin();
    
    /** @brief this function returns the iterator to the last element
     *  of the results map.
     * 
     *  @return iterator to the last element of the results map.
     */
    QMap<QString, QVector<QString>>::iterator end();
    
    /** @brief function which returns the number of topics available
     *  which can be added.
     * 
     *  @return number of available topics
     */
    int count();
    
private:
    ros::master::V_TopicInfo topicMap; /**< map of the topics */
    QRegularExpression regex;/**< regular expression to be matched */
    QMap<QString, QVector<QString>> topics; /**< map with topics as key */
    QString datatype; /**< type of messages expected: "baxter_gbi_input_msgs/signal"*/
    int n_topics; /**< number of topics that can be added */
};
