#include <campusrover_map_server/campusrover_map_server.h>

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string& fname, double res)
    {
      switch_map(fname, res);
      service = n.advertiseService("static_map", &MapServer::mapCallback, this);
    }

    void switch_map(const std::string& fname, double res)
    {
      std::string mapfname = "";
      double origin[3];
      int negate;
      double occ_th, free_th;
      MapMode mode = TRINARY;
      std::string frame_id;
      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", frame_id, std::string("map"));
      deprecated = (res != 0);
      if (!deprecated) {
        //mapfname = fname + ".pgm";
        //std::ifstream fin((fname + ".yaml").c_str());
        std::ifstream fin(fname.c_str());
        if (fin.fail()) {
          ROS_ERROR("Map_server could not open %s.", fname.c_str());
          exit(-1);
        }
#ifdef HAVE_YAMLCPP_GT_0_5_0
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        try {
          doc["resolution"] >> res;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["negate"] >> negate;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain a negate tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["occupied_thresh"] >> occ_th;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["free_thresh"] >> free_th;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
          exit(-1);
        }
        try {
          std::string modeS = "";
          doc["mode"] >> modeS;

          if(modeS=="trinary")
            mode = TRINARY;
          else if(modeS=="scale")
            mode = SCALE;
          else if(modeS=="raw")
            mode = RAW;
          else{
            ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
            exit(-1);
          }
        } catch (YAML::Exception) {
          ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
          mode = TRINARY;
        }
        try {
          doc["origin"][0] >> origin[0];
          doc["origin"][1] >> origin[1];
          doc["origin"][2] >> origin[2];
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain an origin tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["image"] >> mapfname;
          // TODO: make this path-handling more robust
          if(mapfname.size() == 0)
          {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
          }
          if(mapfname[0] != '/')
          {
            // dirname can modify what you pass it
            char* fname_copy = strdup(fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
          }
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain an image tag or it is invalid.");
          exit(-1);
        }
      } else {
        private_nh.param("negate", negate, 0);
        private_nh.param("occupied_thresh", occ_th, 0.65);
        private_nh.param("free_thresh", free_th, 0.196);
        mapfname = fname;
        origin[0] = origin[1] = origin[2] = 0.0;
      }

      ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
      map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;

      //pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1,

      // Latched publisher for metadata
      metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1);
      metadata_pub.publish( meta_data_message_ );

      // Latched publisher for data
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);
      map_pub.publish( map_resp_.map );
    }

  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::ServiceServer service;
    bool deprecated;

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");
      return true;
    }

    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;

    /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }
    */
};

MapServer *ms;

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("table_name", table_name_, std::string("map"));
  n_private.param<string>("file_path", file_path_, std::string("/home"));
}

bool update_map(campusrover_map_server::MapUpdate::Request  &req,
                campusrover_map_server::MapUpdate::Response &res)
{
  string file_path;
  try
  {
    sql::PreparedStatement *sql_pstmt;
    string str_x = to_string(req.utm_x);
    string str_y = to_string(req.utm_y);
    string sql_command = "SELECT * FROM "+ table_name_ +" WHERE min_utm_x <= "
                        + str_x + " AND " + str_x + " <= max_utm_x "
                        + " AND min_utm_y <= " + str_y + " AND "
                        + str_y + " <= max_utm_y";
    sql_pstmt = sql_con_->prepareStatement(sql_command);
    sql_res_ = sql_pstmt->executeQuery();

    /* Fetch in reverse = descending order! */
    sql_res_->afterLast();
    if(sql_res_->previous());
    {
        cout << "\t... MySQL counts: " << sql_res_->getInt("map_id") << endl;
        file_path = file_path_+ "/" + sql_res_->getString("path");
        tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "earth";
        static_transformStamped.child_frame_id = "map";
        static_transformStamped.transform.translation.x = sql_res_->getDouble("tf_x");
        static_transformStamped.transform.translation.y = sql_res_->getDouble("tf_y");
        static_transformStamped.transform.translation.z = 0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, sql_res_->getDouble("tf_yaw"));
        static_transformStamped.transform.rotation.x = quat.x();
        static_transformStamped.transform.rotation.y = quat.y();
        static_transformStamped.transform.rotation.z = quat.z();
        static_transformStamped.transform.rotation.w = quat.w();
        static_broadcaster.sendTransform(static_transformStamped);
        ms->switch_map(file_path, 0);
    }
    // while (sql_res_->previous())
    //   cout << "\t... MySQL counts: " << sql_res_->getInt("map_id") << endl;
    delete sql_res_;
  }
  catch (sql::SQLException &e)
  {
    cout << "# ERR: SQLException in " << __FILE__;
    cout << "(" << __FUNCTION__ << ") on line "
         << __LINE__ << endl;
    cout << "# ERR: " << e.what();
    cout << " (MySQL error code: " << e.getErrorCode();
    cout << ", SQLState: " << e.getSQLState() <<
       " )" << endl;
  }
  return true;
}

void connect_database()
{
  try
  {
    sql_driver_ = get_driver_instance();
    sql_con_ = sql_driver_->connect("tcp://127.0.0.1:3306", "root", "campusrover");
    sql_con_->setSchema("campusrover_database");
  }
  catch (sql::SQLException &e)
  {
    cout << "# ERR: SQLException in " << __FILE__;
    cout << "(" << __FUNCTION__ << ") on line "
         << __LINE__ << endl;
    cout << "# ERR: " << e.what();
    cout << " (MySQL error code: " << e.getErrorCode();
    cout << ", SQLState: " << e.getSQLState() <<
       " )" << endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "campusrover_map_server");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  connect_database();
  std::string fname("/home/justin/campusrover_ws/src/campusrover_summer_demo_2017/campusrover_map_server/maps/mit/media_lab_yard.yaml");
  get_parameters(nh_private);
  ros::ServiceServer service = n.advertiseService("update_map", update_map);
  try
  {
    // Intial static tf
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "earth";
    static_transformStamped.child_frame_id = "map";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
    ms = new MapServer(fname, 0);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }
}
