#include <click_point_rviz.hpp>

namespace pev_rviz_tools
{
  ClickPointTool::ClickPointTool()
  {
    setIcon( rviz::loadPixmap("package://pev_ui_tools/icons/ClickPoint.png") );
    pub_ = nh_.advertise<geometry_msgs::PointStamped>( "/pev_points", 1 );
  }

  ClickPointTool::~ClickPointTool()
  {

  }

  void ClickPointTool::onInitialize()
  {

  }

  void ClickPointTool::activate()
  {

  }

  void ClickPointTool::deactivate()
  {

  }
  int ClickPointTool::processMouseEvent( rviz::ViewportMouseEvent& event )
  {
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    static geometry_msgs::PointStamped point;
    point.header.seq++;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "world";
    if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                           ground_plane,
                                           event.x, event.y, intersection ))
    {
      if( event.leftDown() )
      {
        point.point.x = intersection[0];
        point.point.y = intersection[1];
        point.point.z = intersection[2];
        pub_.publish(point);
      }
      else if(event.middleDown())
      {

      }
    }

  }
} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pev_rviz_tools::ClickPointTool, rviz::Tool )
