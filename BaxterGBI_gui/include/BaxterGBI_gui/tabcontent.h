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
    ~TabContent();

private slots:
    void addMapping();

public slots:
    QVector<QPair<QString, QString>> getTopics();
    void removeMapping(Mapping *mapping);
    void clear();
    void enableAddButton();

private:
    Ui::TabContent *ui;
    QStandardItemModel *model;
    int count;
    
signals:
		void numberOfMappings(int count);
};

#endif // TABCONTENT_H
