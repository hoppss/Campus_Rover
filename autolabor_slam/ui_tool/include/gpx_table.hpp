#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <ctime>
#include "tinyxml.h"

 #include <gps_common/conversions.h>

struct GpxCoordinate
{
  double x;
  double y;
  double z;
};

struct GpxElement
{
  std::string name;
  std::vector<GpxCoordinate> points;
};

enum RadioId {lane_enum, waypoint_enum, object_enum};

class GpxTable
{
public:
  std::string UTM_zone;
  GpxTable();
  ~GpxTable();
  void addSubPoints(double x, double y, double z);
  void setItemTypeLane();
  void setItemTypeWaypoint();
  void setItemTypeObject();
  void insertCurrentItem();
  void deleteLastItem();
  void deleteLastPoint();
  void clearAllItem();
  void saveToFile(std::string file_name);
  void resetAll();
  void getItemsNumber(int &lane_num, int &way_num, int &object_num);
private:
  RadioId item_type_;
  std::vector<GpxElement> lane_vector_, way_vector_, object_vector_;
  GpxElement current_points_;
  void addWaypoint(TiXmlElement *root,
                   std::string name,
                   double lat,
                   double lon,
                   int ele );
  void addTrack(TiXmlElement *root,
                std::string name,
                std::vector<GpxCoordinate> &points);
};
