#ifndef TABCONTENT_H
#define TABCONTENT_H

#include "mapping.h"

#include <QWidget>

namespace Ui {
class TabContent;
}

class TabContent : public QWidget
{
	Q_OBJECT

public:
	explicit TabContent(QStandardItemModel *model, QWidget *parent = nullptr);
	QVector<QPair<QString, QString>> getSelectedTopics();
	void addMapping();
	~TabContent();

public slots:
	void removeMapping(Mapping *mapping);
	void clear();

private:
	Ui::TabContent *ui;
	QVector<Mapping*> mappings;
	QStandardItemModel *model;
	int count;

signals:
	void numberOfMappings(int count);
};

#endif // TABCONTENT_H
