#ifndef TASK_H
#define TASK_H

#include <QWidget>
#include <QString>
#include <QStandardItemModel>

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
	void updateSubtopics(int idx);

private:
	Ui::Mapping* ui;
	QStandardItemModel *model;
};

#endif // TASK_H
