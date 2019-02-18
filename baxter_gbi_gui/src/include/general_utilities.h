/** @file general_utilities.h
 *  @brief Function prototypes for general utilities
 * 
 *  @author Lucrezia Grassi
 *  @author Patrick Roncagliolo
 */ 
#include <QVector>
#include <QString>
#include <QModelIndex>
#include <QStandardItem>
#include <QAbstractItemModel>
#include <QLayout>
#include <vector>
#include <string>

/** @brief converts a QVector of QString in a std::vector
 *  of std::string.
 *  
 *  @param[in] qtvqts QVector of QString
 *  @return this function returns a std::vector of std::string
 */
std::vector<std::string> qt2cxx_strvec(QVector<QString> &qtvqts);

/** @brief converts a std::vector of std::string in a 
 *  QVector of QString.
 *  
 *  @param[in] stdvstds std::vector of std::string
 *  @return this function returns a QVector of QString
 */
QVector<QString> cxx2qt_strvec(std::vector<std::string> &stdvstds);

/** @brief marks if an item is selectable or not.
 * 
 *  @param[in] item pointer to a QStandardItem
 *  @param[in] selectable bool which indicates if the respective item
 * 						is selectable or not
 */
void markSelectable(QStandardItem *item, bool selectable);

/** @brief gets the selectable indexes of the model.
 * 
 *  @param[in] model pointer to a QAbstractItemModel-
 *  @param[in] root root of the sub-tree where is performed the research
 *  					of the selectable item.
 */
QModelIndex getSelectableIdx(QAbstractItemModel *model, const QModelIndex &root = QModelIndex());

/** @brief marks the item with a specific id
 * 
 *  @param[in] item item of the model
 *  @param[in] id identifier of the item
 */
void tagItem(QStandardItem *item, int id);

/** @brief unmarks the item with a specific id
 * 
 *  @param[in] item item of the model
 *  @param[in] id identifier of the item
 */
void untagItem(QStandardItem *item, int id);

/** @brief clears the container of the layout
 *  
 *  @param[in] w pointer to the QLayout container object
 */
void clearContainer(QLayout *w);
