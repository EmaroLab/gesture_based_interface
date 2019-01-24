#ifndef TASK_H
#define TASK_H

#include "BaxterGBI_gui/selectionfiltermodel.h"

#include <QWidget>
#include <QString>
#include <QStandardItemModel>
#include <QStandardItem>

namespace Ui {
class Mapping;
}

class Mapping : public QWidget
{
	Q_OBJECT

public:
	explicit Mapping(QStandardItemModel *model, QWidget* parent = nullptr);
	QPair<QString, QString> currentSelection();
	~Mapping();

	bool isCompleted() const;

signals:
	void removed(Mapping* mapping);

private slots:
    void onTopicChange(int newTopicIdx);
    void onSubtopicChange(int newSubtopicIdx);

private:
    static unsigned int _id;
    unsigned int id = 0;
    int currentTopicIdx = 0;
    int currentSubtopicIdx = 0;
	Ui::Mapping* ui;
    QStandardItemModel *model;
    SelectionFilterModel *filter;
    QStandardItem *topic, *subtopic;
    bool ignoreChange = false;
};

#endif // TASK_H
