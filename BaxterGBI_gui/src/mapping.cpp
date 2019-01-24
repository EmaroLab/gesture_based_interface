#include "BaxterGBI_gui/mapping.h"
#include "ui_mapping.h"

#include <QInputDialog>
#include <QDebug>
#include <QComboBox>

Mapping::Mapping(QStandardItemModel *model, QWidget *parent) :
	QWidget(parent),
	ui(new Ui::Mapping),
    model(model)
    {
		ui->setupUi(this);
		connect(ui->deleteMapping, &QPushButton::clicked, [this] {
						emit removed(this);
						});

		ui->topic->setModel(model);
		ui->subtopic->setModel(model);

        qInfo() << "Mapping()";
        for (currentTopicIdx = 0; currentTopicIdx < model->rowCount(); currentTopicIdx++){
            auto flags = model->item(currentTopicIdx, 0)->flags();
            if (flags & Qt::ItemIsSelectable) break;
        }
        ui->topic->setCurrentIndex(currentTopicIdx);
        auto topic = model->item(currentTopicIdx, 0);

        for (currentSubtopicIdx = 0; currentSubtopicIdx < topic->rowCount(); currentSubtopicIdx++){
            auto flags = topic->child(currentSubtopicIdx, 0)->flags();
            if (flags & Qt::ItemIsSelectable) break;
        }
        ui->subtopic->setRootModelIndex(topic->index());
        ui->subtopic->setCurrentIndex(currentSubtopicIdx);
        auto subtopic = topic->child(currentSubtopicIdx, 0);

        subtopic->setFlags(subtopic->flags() & (~Qt::ItemIsSelectable));
        qInfo() << "\t disabled subtopic " << subtopic->text();
        if (currentSubtopicIdx == topic->rowCount()-1){
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
   auto topic = model->item(currentTopicIdx, 0);
   auto subtopic = topic->child(currentSubtopicIdx, 0);
   topic->setFlags(topic->flags() | Qt::ItemIsSelectable);
   qInfo() << "\t enabled topic " << topic->text();
   subtopic->setFlags(subtopic->flags() | Qt::ItemIsSelectable);
   qInfo() << "\t enabled subtopic " << subtopic->text();
   delete ui;
}

void Mapping::onTopicChange(int newTopicIdx){
    qInfo() << "onTopicChange()";
    auto topic = model->item(currentTopicIdx, 0);
    auto subtopic = topic->child(currentSubtopicIdx, 0);
    topic->setFlags(topic->flags() | Qt::ItemIsSelectable);
    qInfo() << "\t enabled topic " << topic->text();
    subtopic->setFlags(subtopic->flags() | Qt::ItemIsSelectable);
    qInfo() << "\t enabled subtopic " << subtopic->text();
    currentTopicIdx = newTopicIdx;
    currentSubtopicIdx = 0;
    topic = model->item(currentTopicIdx, 0);
    for (currentSubtopicIdx = 0; currentSubtopicIdx < topic->rowCount(); currentSubtopicIdx++){
        auto flags = topic->child(currentSubtopicIdx, 0)->flags();
        if (flags & Qt::ItemIsSelectable) break;
    }
    subtopic = topic->child(currentSubtopicIdx, 0);
    subtopic->setFlags(subtopic->flags() & (~Qt::ItemIsSelectable));
    qInfo() << "\t disabled subtopic " << subtopic->text();
    if (currentSubtopicIdx == topic->rowCount()-1){
        topic->setFlags(topic->flags() & (~Qt::ItemIsSelectable));
        qInfo() << "\t disabled topic " << topic->text();
    }
    ui->subtopic->setRootModelIndex(topic->index());
    ui->subtopic->setCurrentIndex(currentSubtopicIdx);
}

void Mapping::onSubtopicChange(int newSubtopicIdx){
    if (currentSubtopicIdx == newSubtopicIdx) return;
    qInfo() << "onSubtopicChange()";
    auto topic = model->item(currentTopicIdx, 0);
    auto subtopic = topic->child(currentSubtopicIdx, 0);
    subtopic->setFlags(subtopic->flags() | Qt::ItemIsSelectable);
    qInfo() << "\t enabled subtopic " << subtopic->text();
    currentSubtopicIdx = newSubtopicIdx;
    subtopic = topic->child(currentSubtopicIdx, 0);
    subtopic->setFlags(subtopic->flags() & (~Qt::ItemIsSelectable));
    qInfo() << "\t disabled subtopic " << subtopic->text();
}

QPair<QString, QString> Mapping::currentSelection(){
	return {ui->topic->currentText(), ui->subtopic->currentText()};
}


