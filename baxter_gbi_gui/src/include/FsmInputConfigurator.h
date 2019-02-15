/** @file FsmInputConfigurator
 *  @brief Function prototypes for the input configurator of the FSM.
 * 
 *  @author Lucrezia Grassi
 *  @author Patrick Roncagliolo
 */ 
#include <string>
#include "ros/ros.h"
#include "TriggerService.h"
#include <QString>
#include <QVector>
#include <QPair>

class FsmInputConfigurator {
public:
    /** @brief constructor
     * 
     * Creates the constructor by configuring as mask (which must be 
     * passed as parameter) the one formed by the concatenation of 
     * prefix + number + suffix.
     * 
     * @param[in] prefix prefix of the mask
     * @param[in] suffix suffix of the mask
     */
    explicit FsmInputConfigurator(QString prefix, QString suffix);
    
    /** @brief constructor
     * 
     * This constructor creates a configurator of parameters such that
     * its mask is the one specified by the user.
     * @param[in] pattern mask
     */
    explicit FsmInputConfigurator(QString pattern);
    
    /** @brief overloading of () operator 
     * 
     * This function loads on the parameter server the configuration 
     * of a key.
     * @param[in] key key of the map
     * @param[in] topics vector of pairs of string containing topic and
     * subtopic.
     */
    void operator()(int key, QVector<QPair<QString, QString>> topics) const;
    
    /** @brief overloading of () operator
     * 
     * This function loads on the parameter server the configuration 
     * of a key.
     * 
     * @param[in] topics vector of strings
     */
    void operator()(int key, QVector<QString> topics) const;
    
    /** @brief function which triggers the reconfiguration of the FSM.
     */
    void commit();
    
private:
    ros::NodeHandle n; /**< node handle */
    QString mask; /**< mask */
    TriggerService fsmReconfigure; /**< object belonging to TriggerService class */
};
