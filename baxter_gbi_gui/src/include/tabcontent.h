/** @file tabcontent.h
 *  @brief Functions and variables to define the content of a tab in the
 *  configuration panel
 * 
 *  @author Lucrezia Grassi 
 *  @author Patrick Roncagliolo
 */

#ifndef TABCONTENT_H
#define TABCONTENT_H

#include "mapping.h"

#include <QWidget>

namespace Ui {
class TabContent;
}

/** @brief content of each tab of the GUI's configuration panel 
 */
class TabContent : public QWidget
{
	Q_OBJECT

public:
  /** @brief class constructor
   *  
   *  Creates a TabContent object
   *  @param[in] model model of type QStandardItemModel
   *  @param[in] parent parent widget
   */
	explicit TabContent(QStandardItemModel *model, QWidget *parent = nullptr);
	
	/** @brief destructor
	 */
	~TabContent();

  /** @brief gets the selected topics of each mapping object in the tab
   * 
   *  @return a vector of pairs of strings (topic and subtopic)
   */
	QVector<QPair<QString, QString>> getSelectedTopics();
	
	/** @brief adds a mapping to the current tab
	 */
	void addMapping();
	
public slots:
  /** @brief removes a mapping
   *  
   *  @param[in] mapping object that must be deleted
   */
	void removeMapping(Mapping *mapping);
	
	/** @brief clears the content of the tab
	 */
	void clear();

private:
	Ui::TabContent *ui; /**< tab content user interface */
	QVector<Mapping*> mappings; /**< vector of mapping objects */
	QStandardItemModel *model; /**< model with topics and subtopics */
	int count; /**< counter to keep track of the number of mappings in each tab */

signals:
  /** @brief emitted when a mapping is removed from the current tab 
  */
  void mappingRemoved();
  
  /** @brief emitted each time a mapping is added or removed to communicate
   *  the current number of mappings inside the tabs.
   * 
   *  @param[in] count current number of mappings 
   */
	void numberOfMappings(int count);
};

#endif // TABCONTENT_H
