#ifndef TASK_H
#define TASK_H

#include <QWidget>
#include <QString>

namespace Ui {
class Mapping;
}

class Mapping : public QWidget
{
    Q_OBJECT

public:
    explicit Mapping(QWidget* parent = nullptr);
    ~Mapping();

    bool isCompleted() const;

signals:
    void removed(Mapping* mapping);

private:
    Ui::Mapping* ui;
};

#endif // TASK_H
