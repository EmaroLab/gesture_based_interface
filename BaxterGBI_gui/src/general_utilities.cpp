#include "general_utilities.h"
#include <QWidget>

std::vector<std::string> qt2cxx_strvec(QVector<QString> &qtvqts){
  QVector<std::string> v;
  v.reserve(qtvqts.size());
  std::transform(qtvqts.begin(),
                 qtvqts.end(),
                 std::back_inserter(v),
                 [](QString s){return s.toStdString();});
  return v.toStdVector();
}

QVector<QString> cxx2qt_strvec(std::vector<std::string> &stdvstds){
  std::vector<QString> v;
  v.clear();
  v.reserve(stdvstds.size());
  std::transform(stdvstds.begin(),
                 stdvstds.end(),
                 std::back_inserter(v),
                 QString::fromStdString);
  return QVector<QString>::fromStdVector(v);
}

void markSelectable(QStandardItem *item, bool selectable){
  auto flags = item->flags();
  if (selectable)
    flags |= Qt::ItemIsSelectable;
  else
    flags &= ~Qt::ItemIsSelectable;
  item->setFlags(flags);
}

QModelIndex getSelectableIdx(QAbstractItemModel *model, const QModelIndex &root){
  for (int i = 0; i < model->rowCount(root); i++){
    auto flags = model->flags(model->index(i, 0, root));
    if (flags & Qt::ItemIsSelectable)
      return model->index(i, 0, root);
  }
  return {}; //Invalid index
}

void tagItem(QStandardItem *item, int id){
  auto data = item->data(Qt::UserRole);
  if (data.typeName() == 0){ //invalid variant
    item->setData(QVariant(QList<QVariant>{id}), Qt::UserRole);
  } else {
    auto list = data.toList();
    list.append(id);
    item->setData(list, Qt::UserRole);
  }
}

void untagItem(QStandardItem *item, int id){
  auto data = item->data(Qt::UserRole);
  auto list = data.toList();
  list.removeOne(id);
  item->setData(list, Qt::UserRole);
}

void clearContainer(QLayout *w){
  QLayoutItem* item;
  while ((item = w->layout()->takeAt(0))){
    delete item->widget();
    delete item;
  }
}

