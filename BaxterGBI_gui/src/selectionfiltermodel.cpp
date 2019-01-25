#include "BaxterGBI_gui/selectionfiltermodel.h"

SelectionFilterModel::SelectionFilterModel(unsigned int id, QObject *parent)
  : QSortFilterProxyModel(parent)
  , id(id)
{}


bool SelectionFilterModel::filterAcceptsRow(int sourceRow,
                                            const QModelIndex &sourceParent) const
{
  QModelIndex idx = sourceModel()->index(sourceRow, 0, sourceParent);
  auto flags = sourceModel()->flags(idx);
  if (flags & Qt::ItemIsSelectable) return true;

  auto data = sourceModel()->data(idx, Qt::UserRole);
  return data.toList().contains(QVariant(id));
}
