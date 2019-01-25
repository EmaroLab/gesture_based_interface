#include "BaxterGBI_gui/selectionfiltermodel.h"

SelectionFilterModel::SelectionFilterModel(unsigned int id, QObject *parent)
    : QSortFilterProxyModel(parent)
    , id(id)
{}


bool SelectionFilterModel::filterAcceptsRow(int sourceRow,
                                            const QModelIndex &sourceParent) const
{
    QModelIndex index0 = sourceModel()->index(sourceRow, 0, sourceParent);
    auto flags = sourceModel()->flags(index0);
    if (flags & Qt::ItemIsSelectable) return true;

    auto data = sourceModel()->data(index0, Qt::UserRole);
    return data.toList().contains(QVariant(id));
}
