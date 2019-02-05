#include <QVector>
#include <QString>
#include <QModelIndex>
#include <QStandardItem>
#include <QAbstractItemModel>
#include <QLayout>
#include <vector>
#include <string>

std::vector<std::string> qt2cxx_strvec(QVector<QString> &qtvqts);

QVector<QString> cxx2qt_strvec(std::vector<std::string> &stdvstds);

void markSelectable(QStandardItem *item, bool selectable);

QModelIndex getSelectableIdx(QAbstractItemModel *model, const QModelIndex &root = QModelIndex());

void tagItem(QStandardItem *item, int id);

void untagItem(QStandardItem *item, int id);

void clearContainer(QLayout *w);
