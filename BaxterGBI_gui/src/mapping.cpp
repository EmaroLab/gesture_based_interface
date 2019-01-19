#include "BaxterGBI_gui/mapping.h"
#include "ui_mapping.h"

#include <QInputDialog>
#include <QDebug>

Mapping::Mapping(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Mapping)
{
    ui->setupUi(this);
    connect(ui->deleteMapping, &QPushButton::clicked, [this] {
        emit removed(this);
    });
}

Mapping::~Mapping()
{
    qDebug() << "~Mapping() called";
    delete ui;
}
