#ifndef SELECTIONFILTERMODEL_H
#define SELECTIONFILTERMODEL_H

#include <QSortFilterProxyModel>

class SelectionFilterModel : public QSortFilterProxyModel
{
    Q_OBJECT
public:
    SelectionFilterModel(unsigned int id, QObject *parent = 0);
protected:
    virtual bool filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const override;
private:
    unsigned int id;
};

#endif // SELECTIONFILTERMODEL_H
