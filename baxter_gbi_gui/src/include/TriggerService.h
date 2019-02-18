/** @file TriggerService.h
 *  @brief Function prototypes and variables for the TriggerService
 * 
 *  @author Lucrezia Grassi
 *  @author Patrick Roncagliolo
 */
#include <string>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"

/** @brief TriggerService class 
 *  
 *  Contains the prototypes for the trigger service which is 
 *  used to call the service passed as parameter.
 */
class TriggerService {
public:
	/** @brief constructor
	 *  @param[in] srv_name name of the service
	 */
	explicit TriggerService(std::string srv_name);

	/** @brief overloading of () operator
	 * 
	 *  Calls the service
	 */
	void operator()();
	
private:
    ros::NodeHandle n; /**< ros node handle */
    ros::ServiceClient client; /**< client requesting the service */
    std_srvs::Trigger request; /**< request passed as parameter to the call */
    std::string srv_name; /**< name of the service */
};
