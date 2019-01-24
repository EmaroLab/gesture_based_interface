#include "BaxterGBI_gui/mapping.h"
#include "ui_mapping.h"

#include <QInputDialog>
#include <QDebug>
#include <QComboBox>

unsigned int Mapping::_id = 0;

Mapping::Mapping(QStandardItemModel *model, QWidget *parent) :
	QWidget(parent),
	ui(new Ui::Mapping),
    model(model)
    {
        id = _id++;
        filter = new SelectionFilterModel(id);
		ui->setupUi(this);
		connect(ui->deleteMapping, &QPushButton::clicked, [this] {
						emit removed(this);
						});

        filter->setSourceModel(model);
        ui->topic->setModel(filter);
        ui->subtopic->setModel(filter);

        qInfo() << "Mapping()";
        for (currentTopicIdx = 0; currentTopicIdx < filter->rowCount(); currentTopicIdx++){
            auto flags = filter->flags(filter->index(currentTopicIdx, 0));
            if (flags & Qt::ItemIsSelectable) break;
        }
        ui->topic->setCurrentIndex(currentTopicIdx);
        auto topicIdx = filter->index(currentTopicIdx, 0);
        auto data = filter->data(topicIdx, Qt::UserRole);
        if (data.typeName() == 0){ //invalid variant
            filter->setData(topicIdx, QVariant(QList<QVariant>{id}), Qt::UserRole);
        } else {
            auto list = data.toList();
            list.append(id);
            filter->setData(topicIdx, list, Qt::UserRole);
        }

        for (currentSubtopicIdx = 0; currentSubtopicIdx < filter->rowCount(topicIdx); currentSubtopicIdx++){
            auto flags = filter->flags(filter->index(currentSubtopicIdx, 0, topicIdx));
            if (flags & Qt::ItemIsSelectable) break;
        }
        ui->subtopic->setRootModelIndex(topicIdx);
        ui->subtopic->setCurrentIndex(currentSubtopicIdx);
        auto subtopicIdx = filter->index(currentSubtopicIdx, 0, topicIdx);
        data = filter->data(subtopicIdx, Qt::UserRole);
        if (data.typeName() == 0){ //invalid variant
            filter->setData(subtopicIdx, QVariant(QList<QVariant>{id}), Qt::UserRole);
        } else {
            auto list = data.toList();
            list.append(id);
            filter->setData(subtopicIdx, list, Qt::UserRole);
        }
        subtopic = model->itemFromIndex(filter->mapToSource(subtopicIdx));
        topic = model->itemFromIndex(filter->mapToSource(topicIdx));

        subtopic->setFlags(subtopic->flags() & (~Qt::ItemIsSelectable));
        qInfo() << "\t disabled subtopic " << subtopic->text()  << "of topic " << subtopic->parent()->text();
        if (currentSubtopicIdx == filter->rowCount(topicIdx)-1){
            topic->setFlags(topic->flags() & (~Qt::ItemIsSelectable));
            qInfo() << "\t disabled topic " << topic->text();
        }

		connect(ui->topic, 
                qOverload<int>(&QComboBox::currentIndexChanged),
                this,
                &Mapping::onTopicChange);

        connect(ui->subtopic,
                qOverload<int>(&QComboBox::currentIndexChanged),
                this,
                &Mapping::onSubtopicChange);
}

Mapping::~Mapping(){
   qInfo() << "~Mapping()";
   ignoreChange = true;

   auto data = subtopic->data(Qt::UserRole);
   auto list = data.toList();
   list.removeOne(id);
   subtopic->setData(list, Qt::UserRole);
   subtopic->setFlags(subtopic->flags() | Qt::ItemIsSelectable);
   qInfo() << "\t enabled subtopic " << subtopic->text()  << "of topic " << subtopic->parent()->text();

   //if (rowCount(filter->mapFromSource(topic->index())){ //the filter has at least one subtopic
       data = topic->data(Qt::UserRole);
       list = data.toList();
       list.removeOne(id);
       topic->setData(list, Qt::UserRole);
       topic->setFlags(topic->flags() | Qt::ItemIsSelectable);
       qInfo() << "\t enabled topic " << topic->text();
    //}
   delete ui;
}

void Mapping::onTopicChange(int newTopicIdx){
    if (ignoreChange) return;
    if (newTopicIdx == -1) return;
    if (topic == model->itemFromIndex(filter->mapToSource(filter->index(newTopicIdx, 0)))) return;

    qInfo() << "Mapping " << id <<" onTopicChange()";

    topic->setFlags(topic->flags() | Qt::ItemIsSelectable);
    subtopic->setFlags(subtopic->flags() | Qt::ItemIsSelectable);


    auto data = subtopic->data(Qt::UserRole);
    auto list = data.toList();
    list.removeOne(id);
    subtopic->setData(list, Qt::UserRole);
    qInfo() << "\t enabled subtopic " << subtopic->text() << "of topic " << subtopic->parent()->text();

    data = topic->data(Qt::UserRole);
    list = data.toList();
    list.removeOne(id);
    topic->setData(list, Qt::UserRole);
    qInfo() << "\t enabled topic " << topic->text();




    auto topicIdx = filter->index(newTopicIdx, 0);
    data = filter->data(topicIdx, Qt::UserRole);
    if (data.typeName() == 0){ //invalid variant
        filter->setData(topicIdx, QVariant(QList<QVariant>{id}), Qt::UserRole);
    } else {
        auto list = data.toList();
        list.append(id);
        filter->setData(topicIdx, list, Qt::UserRole);
    }

    for (currentSubtopicIdx = 0; currentSubtopicIdx < filter->rowCount(topicIdx); currentSubtopicIdx++){
        auto flags = filter->flags(filter->index(currentSubtopicIdx, 0, topicIdx));
        if (flags & Qt::ItemIsSelectable) break;
    }
    auto subtopicIdx = filter->index(currentSubtopicIdx, 0, topicIdx);
    data = filter->data(subtopicIdx, Qt::UserRole);
    if (data.typeName() == 0){ //invalid variant
        filter->setData(subtopicIdx, QVariant(QList<QVariant>{id}), Qt::UserRole);
    } else {
        auto list = data.toList();
        list.append(id);
        filter->setData(subtopicIdx, list, Qt::UserRole);
    }

    subtopic = model->itemFromIndex(filter->mapToSource(subtopicIdx));
    topic = model->itemFromIndex(filter->mapToSource(topicIdx));

    subtopic->setFlags(subtopic->flags() & (~Qt::ItemIsSelectable));
    qInfo() << "\t disabled subtopic " << subtopic->text() << "of topic " << subtopic->parent()->text();
    if (currentSubtopicIdx == filter->rowCount(topicIdx)-1){
        topic->setFlags(topic->flags() & (~Qt::ItemIsSelectable));
        qInfo() << "\t disabled topic " << topic->text();
    }
    currentTopicIdx = newTopicIdx;
    ui->subtopic->setRootModelIndex(topicIdx);
    ui->subtopic->setCurrentIndex(currentSubtopicIdx);
}

void Mapping::onSubtopicChange(int newSubtopicIdx){
    if (ignoreChange) return;
    if (newSubtopicIdx == -1) return;
    if (subtopic == model->itemFromIndex(filter->mapToSource(filter->index(newSubtopicIdx, 0, filter->mapFromSource(topic->index()))))) return;

    qInfo() << "Mapping " << id <<" onSubtopicChange()";

    auto oldSubtopic = subtopic;

    auto topicIdx = filter->index(currentTopicIdx, 0);
    auto subtopicIdx = filter->index(newSubtopicIdx, 0, topicIdx);

    currentSubtopicIdx = newSubtopicIdx;

    auto data = filter->data(subtopicIdx, Qt::UserRole);
    if (data.typeName() == 0){ //invalid variant
        filter->setData(subtopicIdx, QVariant(QList<QVariant>{id}), Qt::UserRole);
    } else {
        auto list = data.toList();
        list.append(id);
        filter->setData(subtopicIdx, list, Qt::UserRole);
    }
    subtopic = model->itemFromIndex(filter->mapToSource(subtopicIdx));
    subtopic->setFlags(subtopic->flags() & (~Qt::ItemIsSelectable));
    qInfo() << "\t disabled subtopic " << subtopic->text() << "of topic " << subtopic->parent()->text();

    oldSubtopic->setFlags(oldSubtopic->flags() | Qt::ItemIsSelectable);
    data = oldSubtopic->data(Qt::UserRole);
    auto list = data.toList();
    list.removeOne(id);
    oldSubtopic->setData(list, Qt::UserRole);
    qInfo() << "\t enabled subtopic " << oldSubtopic->text() << "of topic " << oldSubtopic->parent()->text();
}

QPair<QString, QString> Mapping::currentSelection(){
	return {ui->topic->currentText(), ui->subtopic->currentText()};
}


