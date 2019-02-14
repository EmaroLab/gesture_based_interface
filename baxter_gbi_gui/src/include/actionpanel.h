/** @file actionpanel.h
 *  @brief Function prototypes for the action panel.
 *
 *  Contains the prototypes for the action panel which 
 *  is the one containing the current state of the Baxter.
 *
 *  @author Lucrezia Grassi
 */
 
#ifndef ACTIONPANEL_H
#define ACTIONPANEL_H

#include <string>
#include <QWidget>

namespace Ui {
class ActionPanel;
}

/** @brief ActionPanel class which contains the main methods and variables to
 *  control the action panel
 */
class ActionPanel : public QWidget
{
	Q_OBJECT

public:
  /** @brief class constructor
   *  @param[in] parent widget
   */
  explicit ActionPanel(QWidget* parent = nullptr);
  /** @brief function which updates the GUI session according to the parameters
   *  received
   * 
   *  @param[in] action current state of the robot. 
   *  According to this parameter the GUI image will be updated.
   *  @param[in] msg message associated with the current state shown 
   *  the image.
   */
  void update(QString action, QString msg);
  /** @brief class destructor
   */
	~ActionPanel();

private:
	Ui::ActionPanel* ui;
};

#endif // ACTIONPANEL_H
