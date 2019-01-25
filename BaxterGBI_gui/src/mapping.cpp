#include "BaxterGBI_gui/mapping.h"
#include "ui_mapping.h"

#include <QInputDialog>
#include <QDebug>
#include <QComboBox>
#include <QtGlobal>

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

void markSelectable(QStandardItem *item, bool selectable){
  auto flags = item->flags();
  if (selectable)
    flags |= Qt::ItemIsSelectable;
  else
    flags &= ~Qt::ItemIsSelectable;
  item->setFlags(flags);
}

QModelIndex getSelectableIdx(QAbstractItemModel *model, const QModelIndex &root = QModelIndex()){
  for (int i = 0; i < model->rowCount(root); i++){
    auto flags = model->flags(model->index(i, 0, root));
    if (flags & Qt::ItemIsSelectable)
      return model->index(i, 0, root);
  }
}

unsigned int Mapping::_id = 0;

Mapping::Mapping(QStandardItemModel *model, QWidget *parent) :
	QWidget(parent),
	ui(new Ui::Mapping),
  model(model)
{
  id = _id++;
  ui->setupUi(this);

  filter = new SelectionFilterModel(id);
  filter->setSourceModel(model);
  ui->topic->setModel(filter);
  ui->subtopic->setModel(filter);

  qInfo() << "Mapping()";

  // Search for selectable topic/usbtopic pair
  topicFilterIdx = getSelectableIdx(filter);
  topic = model->itemFromIndex(filter->mapToSource(topicFilterIdx));

  subtopicFilterIdx = getSelectableIdx(filter, topicFilterIdx);
  subtopic = model->itemFromIndex(filter->mapToSource(subtopicFilterIdx));


  // Mark topic/subtopic with the mapping id
  tagItem(topic, id);
  tagItem(subtopic, id);

  // Update selection in GUI
  ui->topic->setCurrentIndex(topicFilterIdx.row());
  ui->subtopic->setRootModelIndex(topicFilterIdx);
  ui->subtopic->setCurrentIndex(subtopicFilterIdx.row());


  // Mark subtopic (and topic, if no other subtopics are free) as not selectable
  markSelectable(subtopic, false);
  qInfo() << "\t disabled subtopic " << subtopic->text()  << "of topic " << subtopic->parent()->text();

  if (filter->rowCount(topicFilterIdx) == 1){ //no selectable subtopics then
    markSelectable(topic, false);
    qInfo() << "\t disabled topic " << topic->text();
  }

  // Connect callbacks to handle changes in selection
  connect(ui->topic,
          qOverload<int>(&QComboBox::currentIndexChanged),
          this,
          &Mapping::onTopicChange);

  connect(ui->subtopic,
          qOverload<int>(&QComboBox::currentIndexChanged),
          this,
          &Mapping::onSubtopicChange);

  connect(ui->deleteMapping, &QPushButton::clicked,
          [this] {emit removed(this);});
}

Mapping::~Mapping(){
  qInfo() << "~Mapping()";

  markSelectable(subtopic, true);
  qInfo() << "\t enabled subtopic " << subtopic->text()  << "of topic " << subtopic->parent()->text();

  markSelectable(topic, true);
  qInfo() << "\t enabled topic " << topic->text();

  untagItem(subtopic, id);
  untagItem(topic, id);

  delete ui;
}

void Mapping::onTopicChange(int newTopicIdx){
  if (newTopicIdx == -1) return;
  if (topic->index() == filter->mapToSource(filter->index(newTopicIdx, 0))) return;

  qInfo() << "Mapping " << id <<" onTopicChange()";

  markSelectable(topic, true);
  qInfo() << "\t enabled topic " << topic->text();

  markSelectable(subtopic, true);
  qInfo() << "\t enabled subtopic " << subtopic->text() << "of topic " << subtopic->parent()->text();

  untagItem(topic, id);
  untagItem(subtopic, id);

  topicFilterIdx = filter->index(newTopicIdx, 0);
  topic = model->itemFromIndex(filter->mapToSource(topicFilterIdx));

  subtopicFilterIdx = getSelectableIdx(filter, topicFilterIdx);
  subtopic = model->itemFromIndex(filter->mapToSource(subtopicFilterIdx));

  tagItem(topic, id);
  tagItem(subtopic, id);

  markSelectable(subtopic, false);
  qInfo() << "\t disabled subtopic " << subtopic->text()  << "of topic " << subtopic->parent()->text();

  if (filter->rowCount(topicFilterIdx) == 1){ //If this was the last subtopic available...
    markSelectable(topic, false);
    qInfo() << "\t disabled topic " << topic->text();
  }

  ui->subtopic->setRootModelIndex(topicFilterIdx);
  ui->subtopic->setCurrentIndex(subtopicFilterIdx.row());
}

void Mapping::onSubtopicChange(int newSubtopicIdx){
  if (newSubtopicIdx == -1) return;
  topicFilterIdx = filter->mapFromSource(subtopic->parent()->index());
  if (subtopic->index() == filter->mapToSource(filter->index(newSubtopicIdx, 0, topicFilterIdx))) return;

  qInfo() << "Mapping " << id <<" onSubtopicChange()";

  auto oldSubtopic = subtopic;

  subtopicFilterIdx = filter->index(newSubtopicIdx, 0, topicFilterIdx);
  subtopic = model->itemFromIndex(filter->mapToSource(subtopicFilterIdx));

  tagItem(subtopic, id);

  markSelectable(subtopic, false);
  qInfo() << "\t disabled subtopic " << subtopic->text() << "of topic " << subtopic->parent()->text();

  untagItem(oldSubtopic, id);

  markSelectable(oldSubtopic, true);
  qInfo() << "\t enabled subtopic " << oldSubtopic->text() << "of topic " << oldSubtopic->parent()->text();
}

QPair<QString, QString> Mapping::currentSelection(){
	return {ui->topic->currentText(), ui->subtopic->currentText()};
}
