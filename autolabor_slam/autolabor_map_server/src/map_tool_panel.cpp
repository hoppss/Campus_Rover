#include <ros/ros.h>
#include <QWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>

#include <rviz/panel.h>

// namespace autolabor_rviz_panels
namespace autolabor_rviz_panels
{
  class MapToolPanel: public rviz::Panel
  {
    // Q_OBJECT
  public:
    QLineEdit* output_topic_editor_;
    MapToolPanel():
    rviz::Panel()
    {
      // QWidget *window = new QWidget;
      QHBoxLayout* topic_layout = new QHBoxLayout;
      topic_layout->addWidget(new QLabel("How are you!"));
      output_topic_editor_ = new QLineEdit;
      topic_layout->addWidget( output_topic_editor_ );
      if (output_topic_editor_ == NULL)
      {
        ROS_INFO("failed create obj");
      }
      else
      {
        ROS_INFO("obj created");
      }
      // window->setLayout(topic_layout);
      show();
    }
    // protected:

  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autolabor_rviz_panels::MapToolPanel, rviz::Panel)
