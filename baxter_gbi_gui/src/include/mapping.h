/** @file mapping.h
 *  @brief Functions and variables used for the Mapping class
 * 
 *  @author Lucrezia Grassi 
 *  @author Patrick Roncagliolo
 */
#ifndef MAPPING_H
#define MAPPING_H

#include "selectionfiltermodel.h"

#include <QWidget>
#include <QString>
#include <QStandardItemModel>
#include <QStandardItem>

namespace Ui {
class Mapping;
}

/** @brief class Mapping used to define the mappings in the config panel
 */
class Mapping : public QWidget
{
	Q_OBJECT

public:
  /** @brief constructor
   *  
   *  Creates a new mapping and adds it to the model.
   *  @param[in] model model containing all the mappings
   *  @param[in] parent parent widget
   */
	explicit Mapping(QStandardItemModel *model, QWidget* parent = nullptr);
	
	/** @brief destructor
	 */
	~Mapping();
	
	/** @brief current selection of the mapping
	 *  @return current pair of topic and subtopic 
	 */
	QPair<QString, QString> currentSelection();

signals:
  /** @brief signal emitted when a mapping is removed.
   */
	void removed(Mapping* mapping);

private slots:
  /** @brief called when the user selects a topic from the dropdown.
   */
  void onTopicChange(int newTopicIdx);
  
  /** @brief called when the user selects a subtopic from the dropdown.
   */
  void onSubtopicChange(int newSubtopicIdx);

private:
  static unsigned int _id; /**< mapping id */
  unsigned int id = 0; /**< id of the new mapping added */
	Ui::Mapping* ui; /**< user interface mapping object */
  QStandardItemModel *model; /**< pointer to the model */
  SelectionFilterModel *filter; /**< proxy for the model */
  QStandardItem *topic, *subtopic; /** topic and subtopic items */
  QModelIndex topicFilterIdx, subtopicFilterIdx; /**< model index for 
																								the selected topic and subtopic*/
};

#endif // MAPPING_H
