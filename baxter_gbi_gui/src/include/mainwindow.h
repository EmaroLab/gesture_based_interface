/** @file mainwindow.h
 *  @brief Function prototypes and variables for the main window of the GUI
 * 
 *  @author Lucrezia Grassi
 *  @author Patrick Roncagliolo
 */ 
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QKeyEvent>
#include <QString>
#include <QMap>
#include <QSharedPointer>
#include "ros/ros.h"

#include "configpanel.h"
#include "actionpanel.h"
#include "menupanel.h"
#include "rosworker.h"
#include "BaxterDisplay.h"
#include "KeystrokePublisher.h"

namespace Ui {
class MainWindow;
}

/** @brief class of the Main Window of the GUI
 */
class MainWindow : public QMainWindow {
	Q_OBJECT

public:
  /** @brief constructor 
   * 
   *  Concstructs the main window of the GUI
   */
	explicit MainWindow(QWidget *parent = 0);
	
	/** @brief destructor
	 */
	~MainWindow();

protected:
  /** @brief this event handler is called with the given event when 
   *  Qt receives a window close request for a top-level widget from 
   *  the window system. By default, the event is accepted and the 
   *  widget is closed.
   *  
   *  @param[in] event closing of the window 
   */
  virtual void closeEvent(QCloseEvent *event) override;

private:
	Ui::MainWindow *ui; /**< interface */
	QThread *rosThread; /**< thread */
	Worker *worker; /**< worker object */
	ConfigPanel conf_page; /**< configuration panel */
	ActionPanel action_page; /**< action panel */
	MenuPanel menu_page; /**< menu page panel */
	QWidget *current_page; /**< pointer to current panel of the GUI */
	BaxterDisplay display; /**< object of BaxterDisplay class*/
  QMap<int, QSharedPointer<KeystrokePublisher>> map; /**<map of topics and
																												subtopics */

private slots:
  /** @brief shows configuration panel
   */
	void showConfig();
	
	/** @brief shows menu panel 
	 * 
	 *  @param[in] title menu title
	 *  @param[in] options menu options in the scroll area
	 *  @param[in] fixed_options fixed options in the bottom part of the menu
	 *  @param[in] selection selected option of the menu
	 */
  void showMenu(QString &title,
                  QVector<QString> &options,
                  QVector<QString> &fixed_options,
                  int8_t selection);
                  
  /** @brief shows action panel
   * 
   *  @param[in] action name of the action
   *  @param[in] msg message describing the action
   */
  void showAction(QString action, 
                    QString msg);
        
  /** @brief switches the GUI panel 
   * 
   *  @param[in] target_page page which will be shown
   */
  void switchPage(QWidget *target_page);
  
  /** @brief sets the GUI configuration mode
   */
	void __setConfigMode();
	
	/** @brief sets the GUI action mode
	 */
	void __setActionMode();
	
	/** @brief sets the GUI menu mode
	 */
	void __setMenuMode();
	
	/** @brief event handler which receives a key release event.
	 * 
	 *  @param[in] event captured event
	 */
  void keyReleaseEvent(QKeyEvent *event);
};

#endif // MAINWINDOW_H
