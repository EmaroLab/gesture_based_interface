/** @file selectionfiltermodel.h
 *  @brief ....
 * 
 *  @author Lucrezia Grassi 
 *  @author Patrick Roncagliolo 
 */
#ifndef SELECTIONFILTERMODEL_H
#define SELECTIONFILTERMODEL_H

#include <QSortFilterProxyModel>

/** @brief this class ...
 */
class SelectionFilterModel : public QSortFilterProxyModel
{
  Q_OBJECT
  
public:
  /** @brief constructor 
   * 
   *  @param[in] id  ...
   *  @param[in] parent parent object
   */
  SelectionFilterModel(unsigned int id, QObject *parent = 0);
  
protected:
  /** @brief  ...
   *  
   *  @param[in] sourceRow  ...
   *  @param[in] sourceParent ..
   */
  virtual bool filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const override;
  
private:
  unsigned int id; /**< filter id */
};

#endif // SELECTIONFILTERMODEL_H
