#ifndef CONFIGPANEL_H
#define CONFIGPANEL_H

#include "BaxterGBI_gui/tabcontent.h"
#include "mapping.h"
#include <map>
#include <vector>
#include <string>

#include <QWidget>
#include <QVector>
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
	std::map<std::string, std::vector<std::string>> compatibleSubtopics;
	QVector<bool> isFilled;
    int n_topics = 0;

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
