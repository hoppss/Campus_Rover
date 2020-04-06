#include <gpx_table.hpp>

GpxTable::GpxTable()
{
  item_type_ = lane_enum;
}

GpxTable::~GpxTable()
{

}

void GpxTable::addSubPoints(double x, double y, double z)
{
  GpxCoordinate point;
  point.x = x;
  point.y = y;
  point.z = 0;
  current_points_.points.push_back(point);
}

void GpxTable::setItemTypeLane(){ item_type_ = lane_enum; }

void GpxTable::setItemTypeWaypoint(){ item_type_ = waypoint_enum; }

void GpxTable::setItemTypeObject(){ item_type_ = object_enum; }

void GpxTable::insertCurrentItem()
{
  if (current_points_.points.size() > 0)
  {
    switch(item_type_)
    {
      case lane_enum:
        lane_vector_.push_back(current_points_);
      break;
      case waypoint_enum:
        way_vector_.push_back(current_points_);
      break;
      case object_enum:
        object_vector_.push_back(current_points_);
      break;
      default:
      break;
    }
    current_points_.points.clear();
  }
  else
  {
    std::cout << "No Item are able to insert!" << std::endl;
  }
}

void GpxTable::deleteLastItem()
{
  switch(item_type_)
  {
    case lane_enum:
      if (lane_vector_.size() > 0)
      {
        lane_vector_.pop_back();
      }
    break;
    case waypoint_enum:
      if (way_vector_.size() > 0)
      {
        way_vector_.pop_back();
      }
    break;
    case object_enum:
      if(object_vector_.size() > 0)
      {
        object_vector_.pop_back();
      }
    break;
    default:
    break;
  }
}

void GpxTable::deleteLastPoint()
{
  if (current_points_.points.size() > 0)
  {
    current_points_.points.pop_back();
  }
}

void GpxTable::clearAllItem()
{
  switch(item_type_)
  {
    case lane_enum:
      if (lane_vector_.size() > 0)
      {
        lane_vector_.clear();
      }
    break;
    case waypoint_enum:
      if (way_vector_.size() > 0)
      {
        way_vector_.clear();
      }
    break;
    case object_enum:
      if(object_vector_.size() > 0)
      {
        object_vector_.clear();
      }
    break;
    default:
    break;
  }
}

void GpxTable::saveToFile(std::string file_name)
{
  std::cout << "save file: " << file_name<< std::endl;

  TiXmlDocument doc;
  TiXmlElement* msg;
  TiXmlComment * comment;
  std::string s;
  TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "UTF-8", "" );

  TiXmlElement *root = new TiXmlElement("gpx");
  root->SetAttribute("version", "1.0");

    TiXmlElement *name_ele = new TiXmlElement( "name" );
  	TiXmlText *name_text = new TiXmlText( "my gpx" );
    name_ele->LinkEndChild(name_text);
    root->LinkEndChild(name_ele);
    // double gps_lat, gps_long;
    UTM_zone = "51R";
    for (int i = 0; i < lane_vector_.size(); i++)
    {
      double gps_lat, gps_long;
      gps_common::UTMtoLL(lane_vector_[i].points[0].y,
                          lane_vector_[i].points[0].x, UTM_zone, gps_lat, gps_long);
      std::string s =  "PEV PATH " + std::to_string(i);
      addWaypoint(root, s, gps_lat, gps_long, 2372);
      std::string name = lane_vector_[i].name;
      addTrack(root, name, lane_vector_[i].points);
    }

    for (int i = 0; i < way_vector_.size(); i++)
    {
      std::string name = way_vector_[i].name;
      addTrack(root, name, way_vector_[i].points);
    }

    for (int i = 0; i < object_vector_.size(); i++)
    {
      std::string name = object_vector_[i].name;
      addTrack(root, name, object_vector_[i].points);
    }

  doc.LinkEndChild( decl );
  doc.LinkEndChild( root );
  doc.SaveFile(file_name);
}

void GpxTable::resetAll()
{
  lane_vector_.clear();
  way_vector_.clear();
  object_vector_.clear();
}

void GpxTable::getItemsNumber(int &lane_num, int &way_num, int &object_num)
{
  lane_num = lane_vector_.size();
  way_num = way_vector_.size();
  object_num = object_vector_.size();
}

void GpxTable::addWaypoint(TiXmlElement *root,
                      std::string name,
                      double lat,
                      double lon,
                      int ele )
{
  TiXmlElement *wpt_ele = new TiXmlElement( "wpt" );
  wpt_ele->SetAttribute("lat", std::to_string(lat));
  wpt_ele->SetAttribute("lon", std::to_string(lon));
    TiXmlElement *elevation_ele = new TiXmlElement("ele");
      TiXmlText *ele_text = new TiXmlText( std::to_string(ele) );
    TiXmlElement *name_ele = new TiXmlElement("name");
      TiXmlText *name_text = new TiXmlText( name );
    elevation_ele->LinkEndChild(ele_text);
    name_ele->LinkEndChild(name_text);
  wpt_ele->LinkEndChild(elevation_ele);
  wpt_ele->LinkEndChild(name_ele);
  root->LinkEndChild(wpt_ele);
}

void GpxTable::addTrack(TiXmlElement *root,
                        std::string name,
                        std::vector<GpxCoordinate> &points)
{
  auto date = std::chrono::system_clock::now();
  TiXmlElement *trk_ele = new TiXmlElement( "trk" );
    TiXmlElement *name_ele = new TiXmlElement("name");
      TiXmlText *name_text = new TiXmlText("PEV PATH");
    TiXmlElement *number_ele = new TiXmlElement("number");
      TiXmlText *number_text = new TiXmlText(std::to_string(1));
    name_ele->LinkEndChild(name_text);
    number_ele->LinkEndChild(number_text);

    TiXmlElement *trkseg_ele = new TiXmlElement("trkseg");
      // for(int i = 0; i < points.size(); i++)
      int size = points.size();
      TiXmlElement **trkpt_eles = new TiXmlElement*[size];
      double gps_lat, gps_long;
      UTM_zone = "51R";
      for (int i = 0; i < size; i++)
      {
        gps_common::UTMtoLL(points[i].y, points[i].x, UTM_zone, gps_lat, gps_long);
        trkpt_eles[i] = new TiXmlElement("trkpt");
        trkpt_eles[i]->SetAttribute("lat", std::to_string(gps_lat));
        trkpt_eles[i]->SetAttribute("lon", std::to_string(gps_long));
          TiXmlElement *trkpt_name_ele = new TiXmlElement("ele");
            TiXmlText *trkpt_name_text = new TiXmlText(std::to_string(2376));
          TiXmlElement *trkpt_number_ele = new TiXmlElement("time");
            TiXmlText *trkpt_number_text = new TiXmlText("2007-10-14T10:14:08Z");
          trkpt_name_ele->LinkEndChild(trkpt_name_text);
          trkpt_number_ele->LinkEndChild(trkpt_number_text);
        trkpt_eles[i]->LinkEndChild(trkpt_name_ele);
        trkpt_eles[i]->LinkEndChild(trkpt_number_ele);
        trkseg_ele->LinkEndChild(trkpt_eles[i]);
      }
    trk_ele->LinkEndChild(name_ele);
    trk_ele->LinkEndChild(number_ele);
    trk_ele->LinkEndChild(trkseg_ele);
    // wpt_ele->
    root->LinkEndChild(trk_ele);
}
