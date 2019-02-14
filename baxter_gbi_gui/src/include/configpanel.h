/** @file configpanel.h
 *  @brief Function prototypes for the configuration panel.
 *    
 *  Contains the prototypes for the configuration panel which 
 *  is the one used to scan and manage inputs.
 *
 *  @author Lucrezia Grassi
 */
 
#ifndef CONFIGPANEL_H
#define CONFIGPANEL_H

#include "FsmInputConfigurator.h"
#include "TopicScanner.h"
#include "tabcontent.h"
#include "mapping.h"

#include <QWidget>
#include <QVector>
#include <QMap>
#include <QString>
#include <QStandardItemModel>

namespace Ui {
class ConfigPanel;
}

/** @brief ConfigPanel class which contains the main methods and variables to
 *  control the configuration panel
 */
class ConfigPanel : public QWidget
{
	Q_OBJECT

public:
/** @brief constructor
 *  @param[in] parent widget
 */
	explicit ConfigPanel(QWidget* parent = nullptr);
/** @brief destructor
 */
	~ConfigPanel();

private:
	Ui::ConfigPanel* ui; /**< pointer to the ConfigPanel object */
	TabContent *tabs[6]; /**< array containing the pointers to the tabs (one for each input)*/
	QStandardItemModel* model; /**<generic model for storing custom data */
	QVector<bool> isFilled; /**<vector used to check if there are no empty tabs */
  int n_topics = 0; /**<number of topics inside a tab */
  TopicScanner scanner; /**< object of type TopicScanner */
  FsmInputConfigurator fsmInputConfigurator; /**< instance of the class FsmInputConfigurator */

private slots:
	/** @brief function which performs a scan of the topics
	 * 
	 * This function clears the content of each tab, disables the 
	 * Add Mapping button, clears the model and performs the scan by using 
	 * the scanner object. Then it fills the model with the topics found
	 * and enables the Add Mapping button if there are at least 6 inputs.
	 * 
	 */
	void scan(); 
	
  /** @brief function which enables the Load Button
   * 
   * This function checks if there are no empty tabs.
   * If this requirement is satisfied, the Load Button is enabled.
   * @param[in] tab tab on which the check is performed
   * @param[in] count number of mappings in the tab 
   * 
   */
	void enableLoadButton(int tab, int count);
	
	/** @brief function which sends the configuration chosen
	 * 
	 * This function retrieves the list of all the mappings, it writes them 
	 * in the parameter server, then it asks to the FSM (finite state machine)
	 * to refresh the configuration.
	 * 
	 */
	void sendConfig();
	
	/** @brief function which adds a mapping to the active tab
	 * 
	 * A new mapping is added to the active tab of the configuration panel. 
	 * Each time this happens, a check is made to verify if other inputs  
	 * are left, otherwise the Add Mapping button is disabled.
	 * 
	 */
	void addMappingToActiveTab();

signals:
  /** @brief signal sent when the scan is terminated */
	void scan_terminated();
	
	/** @brief signal sent when there are topics available
	 *  @param[in] available bool which indicates the presence 
	 *  of topics available */
	void topicsAvailable(bool available);
};

#endif // CONFIGPANEL_H
