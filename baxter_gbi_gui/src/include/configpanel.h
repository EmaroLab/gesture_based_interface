#ifndef CONFIGPANEL_H
#define CONFIGPANEL_H

#include "FsmInputConfigurator.h"
#include "TopicScanner.h"
#include "tabcontent.h"
#include "mapping.h"

#include <QWidget>
#include <QVector>
#include <QMap>
#include <QString>
#include <QStandardItemModel>

namespace Ui {
class ConfigPanel;
}

class ConfigPanel : public QWidget
{
	Q_OBJECT

public:
	explicit ConfigPanel(QWidget* parent = nullptr);
	~ConfigPanel();

private:
	Ui::ConfigPanel* ui;
	TabContent *tabs[6];
	QStandardItemModel* model;
	QVector<bool> isFilled;
    int n_topics = 0;
    TopicScanner scanner;
    FsmInputConfigurator fsmInputConfigurator;

private slots:
	void scan();
	void enableLoadButton(int tab, int count);
	void sendConfig();
	void addMappingToActiveTab();

signals:
	void scan_terminated();
	void topicsAvailable(bool available);
};

#endif // CONFIGPANEL_H
