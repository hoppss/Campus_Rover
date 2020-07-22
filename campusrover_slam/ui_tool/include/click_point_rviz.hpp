#ifndef CLICK_POINT_TOOL_H
#define CLICK_POINT_TOOL_H
// https://github.com/ros-visualization/visualization_tutorials/tree/kinetic-devel/rviz_plugin_tutorials
#include <iostream>
#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/geometry.h>
#include <rviz/load_resource.h>
#include <rviz/render_panel.h>
#include "rviz/view_controller.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

# include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace autolabor_rviz_tools
{
  class ClickPointTool: public rviz::Tool
  {
    Q_OBJECT
  public:
    ClickPointTool();
    ~ClickPointTool();
    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();

    virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
  };
}
#endif
