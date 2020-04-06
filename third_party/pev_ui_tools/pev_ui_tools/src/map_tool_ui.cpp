#include <map_tool_ui.hpp>
MapToolUi::MapToolUi(int argc, char **argv, QWidget *parent)
  : QWidget(parent),
    m_map_tool(argc, argv)
{
  marker_type_group = new QGroupBox(tr("Maker Types"));
  lane_rdo = new QRadioButton(tr(maker_type_str[0]));
  lane_rdo->setChecked(true);
  waypoint_rdo = new QRadioButton(tr(maker_type_str[1]));
  object_rdo = new QRadioButton(tr(maker_type_str[2]));
  delete_last_point_btn = new QPushButton(tr("Delect Last Point"));
  inser_btn = new QPushButton(tr("Inser Item"));
  clr_all_item_btn = new QPushButton(tr("Clear all Items"));
  delete_last_item_btn = new QPushButton(tr("Detete Last Item"));

  main_layout = new QHBoxLayout();
  left_layout = new QVBoxLayout();
  right_layout = new QVBoxLayout();
  maker_type_groupbox_layout = new QVBoxLayout();
  maker_type_groupbox_layout->addWidget(lane_rdo);
  maker_type_groupbox_layout->addWidget(waypoint_rdo);
  maker_type_groupbox_layout->addWidget(object_rdo);
  marker_type_group->setLayout(maker_type_groupbox_layout);
  marker_type_group->setAlignment(Qt::AlignHCenter);
  left_layout->addWidget(marker_type_group);
  left_layout->addWidget(delete_last_point_btn);
  left_layout->addWidget(inser_btn);
  left_layout->addWidget(clr_all_item_btn);
  left_layout->addWidget(delete_last_item_btn);
  item_summary_form_layout = new QFormLayout;
  setSumaryForm(item_summary_form_layout);
  right_layout->addLayout(item_summary_form_layout);
  save_file_btn = new QPushButton(tr("Save to File"));
  reset_table_btn = new QPushButton(tr("Reset all item in table"));
  right_layout->addWidget(save_file_btn);
  right_layout->addWidget(reset_table_btn);
  main_layout->addLayout(left_layout);
  main_layout->addLayout(right_layout);
  setLayout(main_layout);
  show();
  setWindowTitle(tr("PEV Map Tool"));
  connectWidgets();
  m_map_tool.init();
}

MapToolUi::~MapToolUi()
{}

void MapToolUi::setSumaryForm(QFormLayout *form_layout)
{
  item0_summary_lab = new QLabel(tr("0"));
  item1_summary_lab = new QLabel(tr("0"));
  item2_summary_lab = new QLabel(tr("0"));
  item_summary_form_row1_name = new QLabel(tr("Number"));
  form_layout->addRow(tr("Item Type"), item_summary_form_row1_name);
  form_layout->addRow(tr(maker_type_str[0]), item0_summary_lab);
  form_layout->addRow(tr(maker_type_str[1]), item1_summary_lab);
  form_layout->addRow(tr(maker_type_str[2]), item2_summary_lab);
}

void MapToolUi::connectWidgets()
{
  connect(delete_last_point_btn, SIGNAL (released()), this, SLOT (delectLastPoint()));
  connect(inser_btn, SIGNAL (released()), this, SLOT (inserItem()));
  connect(clr_all_item_btn, SIGNAL (released()), this, SLOT (clearAllItem()));
  connect(delete_last_item_btn, SIGNAL (released()), this, SLOT (deleteLastItem()));
  connect(save_file_btn, SIGNAL (released()), this, SLOT (saveToFile()));
  connect(reset_table_btn, SIGNAL (released()), this, SLOT (Reset_table()));
  // Radio button
  connect(lane_rdo, SIGNAL (released()), this, SLOT (chageMarkerType()));
  connect(waypoint_rdo, SIGNAL (released()), this, SLOT (chageMarkerType()));
  connect(object_rdo, SIGNAL (released()), this, SLOT (chageMarkerType()));
}

void MapToolUi::chageMarkerType()
{
  if(lane_rdo->isChecked())
  {
    m_map_tool.setLaneMode();
  }
  else if(waypoint_rdo->isChecked())
  {
    m_map_tool.setWayointMode();
  }
  else if(object_rdo->isChecked())
  {
    m_map_tool.setObjectMode();
  }
}

void MapToolUi::delectLastPoint()
{
  m_map_tool.delectLastPoint();
}

void MapToolUi::inserItem()
{
  m_map_tool.inserItem();
  updateForm();
}

void MapToolUi::deleteLastItem()
{
  m_map_tool.deleteLastItem();
  updateForm();
}

void MapToolUi::clearAllItem()
{
  m_map_tool.clearAllItem();
  updateForm();
}

void MapToolUi::saveToFile()
{
  QString fileName = QFileDialog::getSaveFileName(this,
    tr("GPX"), "",
    tr("GPX files (*.gpx);"));
  m_map_tool.saveToFile(fileName.toUtf8().constData() + std::string(".gpx"));
}

void MapToolUi::Reset_table()
{
  m_map_tool.Reset_table();
  updateForm();
}

void MapToolUi::updateForm()
{
  int lane_num, way_num, object_num;
  m_map_tool.getItemsNumber(lane_num, way_num, object_num);
  item0_summary_lab->setText(QString::number(lane_num));
  item1_summary_lab->setText(QString::number(way_num));
  item2_summary_lab->setText(QString::number(object_num));
}
