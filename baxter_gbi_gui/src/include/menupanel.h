/** @file menupanel.h
 *  @brief Functions and variables to define the MenuPanel class
 *  
 *  @author Lucrezia Grassi 
 *  @author Patrick Roncagliolo 
 */
#ifndef MENUPANEL_H
#define MENUPANEL_H

#include <QWidget>
#include <QPushButton>
#include <QVector>

namespace Ui {
class MenuPanel;
}

/** @brief MenuPanel class to display the GUI menu
 */ 
class MenuPanel : public QWidget
{
	Q_OBJECT

public:
  /** @brief constructor
   * 
   *  Creates a MenuPanel object to display a generic menu page
   */
	explicit MenuPanel(QWidget* parent = nullptr);
	
	/** @brief destructor
	 */
	~MenuPanel();
	
	/** @brief updates the menu page on the basis of the parameters contained 
	 *  in the received message.
	 * 
	 *  @param[in] title menu page title
	 *  @param[in] options options of the menu in the scroll area
	 *  @param[in] fixed_options menu fixed options in the bottom area
	 *  @param[in] selection selected element of the menu
	 */
  void update(QString &title,
              QVector<QString> &options,
              QVector<QString> &fixed_options,
              int8_t selection);

private:
	Ui::MenuPanel* ui; /**< user interface of the Menu panel */
	QVector<QPushButton*> optionsButtons; /**< buttons of the menu */
};

#endif // MENUPANEL_H
