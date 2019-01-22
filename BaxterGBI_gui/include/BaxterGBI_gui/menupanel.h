#ifndef MENUPANEL_H
#define MENUPANEL_H

#include <QWidget>

namespace Ui {
class MenuPanel;
}

class MenuPanel : public QWidget
{
    Q_OBJECT

public:
    explicit MenuPanel(QWidget* parent = nullptr);
    ~MenuPanel();

private:
    Ui::MenuPanel* ui;

private slots:

signals:
 
};

#endif // MENUPANEL_H
