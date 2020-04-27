#include <autolabor_map_server/map_tf_finder.h>

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<double>("tanslation_x", tanslation_x_, 328150.66);
  n_private.param<double>("tanslation_y", tanslation_y_, 4691918.72);
}

// v . w = ||v|| ||w|| cos q
double getTheta(Point &v, Point &w)
{
  double dot_product = v.x * w.x + v.y * w.y;
  double v_r = sqrt(pow(v.x, 2) + pow(v.y, 2));
  double w_r = sqrt(pow(w.x, 2) + pow(w.y, 2));
  return acos(dot_product / (v_r * w_r));
}

void get_tf_callback(const geometry_msgs::PointStampedConstPtr &msg)
{
  static int seq = 1;
  static Point v, w, pt;
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  static double theta, dx, dy;
  geometry_msgs::TransformStamped static_transformStamped;
  nav_msgs::Path path;
  if(seq == 1)
  {
    v.x = msg->point.x;
    v.y = msg->point.y;
  }
  else if(seq == 2)
  {
    v.x -= msg->point.x;
    v.y -= msg->point.y;
  }
  else if(seq == 3)
  {
    w.x = msg->point.x;
    w.y = msg->point.y;
  }
  else if(seq == 4)
  {
    w.x -= msg->point.x;
    w.y -= msg->point.y;
    theta = getTheta(v, w);
    // cout << "Theta: " << theta << endl;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "local_map";
    static_transformStamped.child_frame_id = "map";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
  }
  else if(seq == 5)
  {
    pt.x = msg->point.x;
    pt.y = msg->point.y;
  }
  else if(seq == 6)
  {
    dx =  msg->point.x - pt.x;
    dy =  msg->point.y - pt.y;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "local_map";
    static_transformStamped.child_frame_id = "map";
    static_transformStamped.transform.translation.x = dx;
    static_transformStamped.transform.translation.y = dy;
    static_transformStamped.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
    tf_x_ = tanslation_x_ + dx;
    tf_y_ = tanslation_y_ + dy;
    tf_yaw_ = theta;
    cout << fixed << "Theta: " << tf_yaw_ << endl;
    cout << fixed << "Translation: " << tf_x_ << ", " << tf_y_ << endl;
  }
  else if(seq == 7)
  {
    map_roi[0].x = msg->point.x;
    map_roi[0].y = msg->point.y;
  }
  else if(seq == 8)
  {
    if(map_roi[0].x > msg->point.x)
    {
      map_roi[1].x = map_roi[0].x;
      map_roi[0].x = msg->point.x;
    }
    else
    {
      map_roi[1].x = msg->point.x;
    }

    if(map_roi[0].y > msg->point.y)
    {
      map_roi[1].y = map_roi[0].y;
      map_roi[0].y = msg->point.y;
    }
    else
    {
      map_roi[1].y = msg->point.y;
    }
    map_roi[0].x += tanslation_x_;
    map_roi[1].x += tanslation_x_;
    map_roi[0].y += tanslation_y_;
    map_roi[1].y += tanslation_y_;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "earth";
    addPathPoint(map_roi[0].x, map_roi[0].y, path);
    addPathPoint(map_roi[0].x, map_roi[1].y, path);
    addPathPoint(map_roi[1].x, map_roi[1].y, path);
    addPathPoint(map_roi[1].x, map_roi[0].y, path);
    addPathPoint(map_roi[0].x, map_roi[0].y, path);
    ROI_path_pub_.publish(path);
  }
  seq++;
}

void addPathPoint(double x, double y, nav_msgs::Path &path)
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "earth";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  pose.pose.orientation.x = quat.x();
  pose.pose.orientation.y = quat.y();
  pose.pose.orientation.z = quat.z();
  pose.pose.orientation.w = quat.w();
  path.poses.push_back(pose);
}

bool database_Write(autolabor_map_server::DatabaseWrite::Request  &req,
                    autolabor_map_server::DatabaseWrite::Response &res)
{
  cout << "write: " << endl;
  res.max_utm_x = map_roi[1].x;
  res.min_utm_x = map_roi[0].x;
  res.max_utm_y = map_roi[1].y;
  res.min_utm_y = map_roi[0].y;
  res.tf_x = tf_x_;
  res.tf_y = tf_y_;
  res.tf_yaw = tf_yaw_;
  res.path = req.path;
  try
  {
    sql::Driver *driver;
    sql::Connection *con;
    sql::PreparedStatement *pstmt;
    driver = get_driver_instance();
    con = driver->connect("tcp://127.0.0.1:3306", req.user, req.password);
    cout << "Login autolabor database with " << req.user << endl;
    con->setSchema("autolabor_database");
    pstmt = con->prepareStatement("INSERT INTO map(max_utm_x, min_utm_x, \
       max_utm_y, min_utm_y, tf_x, tf_y, tf_yaw, path) VALUES (?,?,?,?,?,?,?,?)");
    pstmt->setDouble(1, map_roi[1].x);
    pstmt->setDouble(2, map_roi[0].x);
    pstmt->setDouble(3, map_roi[1].y);
    pstmt->setDouble(4, map_roi[0].y);
    pstmt->setDouble(5, tf_x_);
    pstmt->setDouble(6, tf_y_);
    pstmt->setDouble(7, tf_yaw_);
    pstmt->setString(8, req.path);
    pstmt->executeUpdate();
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autolabor_map_tf_finder");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ROI_path_pub_ = n.advertise<  nav_msgs::Path> ("roi_path", 10);
  ros::ServiceServer service = n.advertiseService("save_map_info", database_Write);
  ros::Subscriber point_sub = n.subscribe("clicked_point", 10, get_tf_callback);
  // Intial static tf
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "earth";
  static_transformStamped.child_frame_id = "local_map";
  static_transformStamped.transform.translation.x = tanslation_x_;
  static_transformStamped.transform.translation.y = tanslation_y_;
  static_transformStamped.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
  ros::spin();
  return 0;
}
