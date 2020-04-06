#ifndef MAP_TOOL_UI_H
#define MAP_TOOL_UI_H
#include <QtWidgets>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QRadioButton>
#include <QPushButton>
#include <QFormLayout>
#include <QLabel>
#include <QString>
#include <iostream>
#include "map_tool.hpp"

class MapToolUi : public QWidget
{
  Q_OBJECT
public:
  // variables
  char maker_type_str[3][10] = {"Lane", "Way Point", "Object"};
  // functions
  MapToolUi(int argc, char **argv, QWidget *parent = 0);
  virtual ~MapToolUi();

private:
  // variables
  QWidget *window;
  QHBoxLayout *main_layout;
  QVBoxLayout *left_layout;
  QVBoxLayout *right_layout;
  QVBoxLayout *maker_type_groupbox_layout;
  QGroupBox *marker_type_group;
  QRadioButton *lane_rdo;
  QRadioButton *waypoint_rdo;
  QRadioButton *object_rdo;
  QPushButton *delete_last_point_btn;
  QPushButton *inser_btn;
  QPushButton *clr_all_item_btn;
  QPushButton *delete_last_item_btn;
  // Summary table
  QLabel *item0_summary_lab, *item1_summary_lab, *item2_summary_lab;
  QLabel *item_summary_form_row1_name;
  QFormLayout *item_summary_form_layout;
  QPushButton *save_file_btn;
  QPushButton *reset_table_btn;
  // class
  MapTool m_map_tool;
  // functions
  void setSumaryForm(QFormLayout *form_layout);
  void connectWidgets();
  Q_SLOT void chageMarkerType();
  Q_SLOT void delectLastPoint();
  Q_SLOT void inserItem();
  Q_SLOT void deleteLastItem();
  Q_SLOT void clearAllItem();
  Q_SLOT void saveToFile();
  Q_SLOT void Reset_table();
  void updateForm();
};
#endif
