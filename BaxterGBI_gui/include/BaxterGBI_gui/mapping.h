#ifndef TASK_H
#define TASK_H

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
    int currentTopicIdx = 0;
    int currentSubtopicIdx = 0;
	Ui::Mapping* ui;
    QStandardItemModel *model;
};

#endif // TASK_H
