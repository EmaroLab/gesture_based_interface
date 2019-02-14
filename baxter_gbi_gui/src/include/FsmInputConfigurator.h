/** @file FsmInputConfigurator
 *  @brief Function prototypes for the input configurator of the FSM
 * 
 *  @author Lucrezia Grassi
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
     * This constructor calls another constructor which takes as parameter
     * the mask, obtained as concatenatenation of the prefix and the suffix.
     * @param[in] prefix prefix of the mask
     * @param[in] suffix suffix of the mask
     */
    explicit FsmInputConfigurator(QString prefix, QString suffix);
    /** @brief constructor
     * 
     * This constructor initializes the mask and reconfigures the FSM
     * @param[in] pattern mask
     */
    explicit FsmInputConfigurator(QString pattern);
    
    /** @brief overloading of () operator 
     * 
     * This function concatenates in a string topic and subtopic and 
     * puts these strings in an array.
     * @param[in] key key of the map
     * @param[in] topics vector of pairs of string containing topic and
     * subtopic.
     */
    void operator()(int key, QVector<QPair<QString, QString>> topics) const;
    
    /** @brief overloading of () operator
     * 
     * This function updates the parameter server.
     * @param[in] topics vector of strings
     */
    void operator()(int key, QVector<QString> topics) const;
    
    /** @brief function which reconfigures the FSM.
     */
    void commit();
    
private:
    ros::NodeHandle n; /**<node handle */
    QString mask; /**<mask */
    TriggerService fsmReconfigure; /**< object belonging to TriggerService class */
};
