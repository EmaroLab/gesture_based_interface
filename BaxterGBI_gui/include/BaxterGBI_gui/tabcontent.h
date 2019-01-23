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
    ~TabContent();

private slots:
    void addMapping();

public slots:
    void removeMapping(Mapping *mapping);
    void clear();
    void enableAddButton(bool enable);

private:
    Ui::TabContent *ui;
    QVector<Mapping*> mappings;
    QStandardItemModel *model;
    int count;
    
signals:
		void numberOfMappings(int count);
};

#endif // TABCONTENT_H
