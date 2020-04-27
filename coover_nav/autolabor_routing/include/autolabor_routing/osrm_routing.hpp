#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <utility>

#include <cstdlib>

// LIBRARIES
#include <gps_common/conversions.h>

struct RouteCoordinate {
  double lat;
  double lon;
};

using namespace osrm;

class OsrmRouting {
  public:
    RouteParameters params;
    const OSRM *osrm;
    OsrmRouting(std::string osrm_file);
    ~OsrmRouting()
    {
      delete osrm;
    }
    void getRoute(RouteParameters &params);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gpsDestinationCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void updateAutolaborGpsLocationCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
};
void get_parameters(ros::NodeHandle n_private);
void upSampling(bool new_route,
                nav_msgs::Path &path,
                double new_poiunt_x,
                double new_poiunt_y,
                double resolution_dxy);

void pushPointToPath(bool new_route,
                    nav_msgs::Path &path,
                    double new_poiunt_x,
                    double new_poiunt_y);

std::string path_frame_id_, osrm_file_, UTM_zone_;
ros::Publisher route_path_pub_;
RouteCoordinate origin, destination;
double route_resolution_;
