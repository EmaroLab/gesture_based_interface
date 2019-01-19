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
    explicit TabContent(QWidget *parent = nullptr);
    ~TabContent();

private slots:
    void addMapping();

public slots:
    QVector<QPair<QString, QString>> getTopics();
    void removeMapping(Mapping* mapping);

private:
    Ui::TabContent *ui;
};

#endif // TABCONTENT_H
