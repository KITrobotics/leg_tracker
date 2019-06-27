
#include <leg_tracker/leg_tracker.h>

LegDetector::LegDetector(ros::NodeHandle nh) : nh_(nh), tfListener(tfBuffer)
{
    init();
}

void LegDetector::init()
{ 
    std::srand(1);
    resetLeftRight();

    nh_.param("scan_topic", scan_topic, std::string("/scan_unified"));
    nh_.param("frequency", frequency, 0.05);
    nh_.param("transform_link", transform_link, std::string("base_link"));
    nh_.param("global_map_topic", global_map_topic, std::string("/move_base/global_costmap/costmap"));
    nh_.param("x_lower_limit", x_lower_limit, 0.0);
    nh_.param("x_upper_limit", x_upper_limit, 0.5);
    nh_.param("y_lower_limit", y_lower_limit, -0.5);
    nh_.param("y_upper_limit", y_upper_limit, 0.5);
    x_lower_limit_dynamic = x_lower_limit;
    x_upper_limit_dynamic = x_upper_limit;
    y_lower_limit_dynamic = y_lower_limit;
    y_upper_limit_dynamic = y_upper_limit;
    nh_.param("leg_radius", leg_radius, 0.1);
    nh_.param("ref_point_x", ref_point_x, -0.9);
    nh_.param("ref_point_y", ref_point_y, 0.0);
    nh_.param("min_observations", min_observations, 4);
    nh_.param("max_dist_btw_legs", max_dist_btw_legs, 0.8);
    nh_.param("z_coordinate", z_coordinate, 0.178);
    nh_.param("vel_stance_threshold", vel_stance_threshold, 0.47);
    nh_.param("vel_swing_threshold", vel_swing_threshold, 0.93);
    nh_.param("state_dimensions", state_dimensions, 6);
    nh_.param("minClusterSize", minClusterSize, 3);
    nh_.param("maxClusterSize", maxClusterSize, 100);
    nh_.param("clusterTolerance", clusterTolerance, 0.07);
    nh_.param("isOnePersonToTrack", isOnePersonToTrack, false);
    nh_.param("isBoundingBoxTracking", isBoundingBoxTracking, false);
    nh_.param("with_map", with_map, false);
    nh_.param("occluded_dead_age", occluded_dead_age, 10);
    nh_.param("variance_observation", variance_observation, 0.25);
    nh_.param("min_dist_travelled", min_dist_travelled, 0.25);
    nh_.param("max_cov", max_cov, 0.81);
    nh_.param("in_free_space_threshold", in_free_space_threshold, 0.06);
    
    nh_.param("mahalanobis_dist_gate", mahalanobis_dist_gate, 1.2);
    nh_.param("euclidian_dist_gate", euclidian_dist_gate, 0.4);
    nh_.param("max_cost", max_cost, 999999.);
    nh_.param("tracking_bounding_box_uncertainty", tracking_bounding_box_uncertainty, 0.2);
    nh_.param("cluster_bounding_box_uncertainty", cluster_bounding_box_uncertainty, 0.03);
    nh_.param("outlier_removal_radius", outlier_removal_radius, 0.07);
    nh_.param("max_neighbors_for_outlier_removal", max_neighbors_for_outlier_removal, 3);
    
    legs_gathered = id_counter = legs_marker_next_id = next_leg_id = people_marker_next_id = 
	cov_ellipse_id = 0;
    got_map = false;
    got_map_from_service = false;

    sub = nh_.subscribe<sensor_msgs::LaserScan>(scan_topic, 1, &LegDetector::processLaserScan, this);
    global_map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>(global_map_topic, 10, &LegDetector::globalMapCallback, this);
    pos_vel_acc_fst_leg_pub = nh_.advertise<leg_tracker::LegTrackerMessage>("posXY_velXY_accXY_lId_pId_conf_fst_leg", 300);
    pos_vel_acc_snd_leg_pub = nh_.advertise<leg_tracker::LegTrackerMessage>("posXY_velXY_accXY_lId_pId_conf_snd_leg", 300);
    legs_and_vel_direction_publisher = nh_.advertise<visualization_msgs::MarkerArray>("legs_and_vel_direction", 300);
    tracking_area_pub = nh_.advertise<visualization_msgs::Marker>("tracking_area", 300);
    people_pub = nh_.advertise<visualization_msgs::MarkerArray>("people", 300);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("line_strip", 10);
    cov_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("cov_ellipses", 10);
    
    people_msg_pub = nh_.advertise<leg_tracker::PersonMsg>("people_msg_stamped", 10);
    
//     bounding_box_pub = nh_.advertise<visualization_msgs::Marker>("bounding_box", 300);
    tracking_zone_pub = nh_.advertise<visualization_msgs::MarkerArray>("tracking_zones", 100);
//     paths_publisher = nh_.advertise<visualization_msgs::MarkerArray>("paths", 100);
//     client = nh_.serviceClient<nav_msgs::GetMap>("static_map");
  }
  
  
  double LegDetector::getRandomNumberFrom0To1()
  {
    return (double)rand() / (double)RAND_MAX ;
  }
  
  
  void LegDetector::globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
  {
    global_map = *msg;
    if (!got_map) { got_map = true; }
  }
  

  double LegDetector::calculateNorm(Point p)
  {
    return std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));
  }
  

  visualization_msgs::Marker LegDetector::getCovarianceEllipse(int id, const double meanX, const double meanY, const Eigen::MatrixXd& S)
  {

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(S);
    double l1 = solver.eigenvalues().x();
    double l2 = solver.eigenvalues().y();
    Eigen::VectorXd e1 = solver.eigenvectors().col(0);
    Eigen::VectorXd e2 = solver.eigenvectors().col(1);

    double scale95 = std::sqrt(5.991);
    double R1 = scale95 * std::sqrt(l1);
    double R2 = scale95 * std::sqrt(l2);
    double tilt = std::atan2(e2.y(), e2.x());


    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitZ());
    double orientation_w = q.w();
    double orientation_x = q.x();
    double orientation_y = q.y();
    double orientation_z = q.z();
    
    return getOvalMarker(id, meanX, meanY, orientation_x, orientation_y, orientation_z,
      orientation_w, R1, R2, 0.0, 0.0, 1.0);
  }


  bool LegDetector::laserScanToPointCloud2(const sensor_msgs::LaserScan::ConstPtr& scan, sensor_msgs::PointCloud2& cloud)
  {
    if (!scan) { ROS_DEBUG("Laser scan pointer was not set!"); return false; }
    projector_.projectLaser(*scan, cloud);
    return true;
  }

  
  bool LegDetector::tfTransformOfPointCloud2(const sensor_msgs::LaserScan::ConstPtr& scan,
				sensor_msgs::PointCloud2& from, sensor_msgs::PointCloud2& to)
  {
    geometry_msgs::TransformStamped transformStamped;
    try{
      std::string frame_id_string = scan->header.frame_id;
      char firstChar = frame_id_string[0];
      if (firstChar == '/') 
      {
	frame_id_string = frame_id_string.replace(0, 1, "");
      }
      transformStamped = tfBuffer.lookupTransform(transform_link, frame_id_string, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }
    tf2::doTransform(from, to, transformStamped);
    return true;
  }
  
  void LegDetector::pub_leg_posvelacc(std::vector<double>& in, bool isSnd, std_msgs::Header header)
  {
    // 3 values for legId peopleId and confidence
    if (in.size() != 6 + 3) { ROS_ERROR("Invalid vector of leg posvelacc!"); return; }

    leg_tracker::LegTrackerMessage msg;
    msg.header = header;

    std_msgs::Float64MultiArray array;

    // set up dimensions
    array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    array.layout.dim[0].size = in.size();
    array.layout.dim[0].stride = 1;
    array.layout.dim[0].label = "posXY_velXY_accXY_lId_pId_confidence";

    // copy in the data
    array.data.clear();
    array.data.insert(array.data.end(), in.begin(), in.end());

    msg.array = array;

    if (isSnd) { pos_vel_acc_fst_leg_pub.publish(msg); }
    else { pos_vel_acc_snd_leg_pub.publish(msg); }
  }

  // pass through filtering, outlier removal
  bool LegDetector::filterPCLPointCloud(const PointCloud& in, PointCloud& out)
  {
    if (in.points.size() < minClusterSize)
    {
      ROS_DEBUG("Filtering: Too small number of points in the input PointCloud!");
      return false;
    }
    
    out.header = in.header;
    
    PointCloud pass_through_filtered_x;
    PointCloud pass_through_filtered_y;
    PointCloud outlier_filtered;

    pcl::PassThrough<Point> pass;
    pass.setInputCloud(in.makeShared());
    pass.setFilterFieldName("x");
    if (isOnePersonToTrack) {
      pass.setFilterLimits(x_lower_limit_dynamic, x_upper_limit_dynamic);
    } else {
      pass.setFilterLimits(x_lower_limit, x_upper_limit);
    }
    pass.setFilterLimitsNegative (false);
    pass.filter ( pass_through_filtered_x );
    if ( pass_through_filtered_x.points.size() < minClusterSize)
    {
      ROS_DEBUG("Filtering: Too small number of points in the PointCloud after PassThrough filter in x direction!");
      return false;
    }
    pass.setInputCloud( pass_through_filtered_x.makeShared());
    pass.setFilterFieldName("y");
    if (isOnePersonToTrack) {
      pass.setFilterLimits(y_lower_limit_dynamic, y_upper_limit_dynamic);
    } else {
      pass.setFilterLimits(y_lower_limit, y_upper_limit);
    }
    pass.filter (pass_through_filtered_y);
    if ( pass_through_filtered_y.points.size() < minClusterSize)
    {
      ROS_DEBUG("Filtering: Too small number of points in the PointCloud after PassThrough filter in y direction!");
      return false;
    }

    pcl::RadiusOutlierRemoval<Point> outrem;
    outrem.setInputCloud(pass_through_filtered_y.makeShared());
    outrem.setRadiusSearch(outlier_removal_radius);
    outrem.setMinNeighborsInRadius (max_neighbors_for_outlier_removal);
    
    if (with_map)
    {
      outrem.filter(outlier_filtered);
      if (outlier_filtered.points.size() < minClusterSize)
      {
	ROS_DEBUG("Filtering: Too small number of points in the resulting PointCloud!");
	return false;
      }
      
      if (got_map)
      {
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::PointStamped point_in, point_out;
	point_in.header.frame_id = in.header.frame_id;
	point_in.header.stamp = ros::Time::now();
	
	try{
	  transformStamped = tfBuffer.lookupTransform(global_map.header.frame_id, point_in.header.frame_id, ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
	  ROS_WARN("Failure to lookup the transform for a point! %s\n", ex.what());
	  return false;
	}
	
	
	for (int i = 0; i < outlier_filtered.points.size(); i++) 
	{
	  point_in.point.x = outlier_filtered.points[i].x; 
	  point_in.point.y = outlier_filtered.points[i].y;
	  
	  tf2::doTransform(point_in, point_out, transformStamped);
	  
	  double in_free_space = how_much_in_free_space(point_out.point.x, point_out.point.y);
	  
	  if (in_free_space <= in_free_space_threshold) 
	  {
	    out.points.push_back(outlier_filtered.points[i]);
	  } 
	}
      }
    }
    else
    {
      outrem.filter(out);
      if (out.points.size() < minClusterSize)
      {
	ROS_DEBUG("Filtering: Too small number of points in the resulting PointCloud!");
	return false;
      }
    }
    
    return true;
  }

  
  Leg LegDetector::initLeg(const Point& p)
  {
    Leg l(getNextLegId(), p, occluded_dead_age,
      variance_observation, min_observations, state_dimensions, min_dist_travelled);
    return l;
  }


  void LegDetector::printLegsInfo(std::vector<Leg> vec, std::string name)
  {
    ROS_INFO("%s:", name.c_str());
    for (Leg& l : vec)
    {
      double confidence = l.hasPair() * std::max(0., (1 - 0.15 * l.getOccludedAge()));
      std::string left_or_right_leg = "unknown"; 
      if (left_right.first != -1)
      {
	left_or_right_leg = (left_right.first == l.getLegId() ? "left" : "right");
      }
      
      double angle = atan2(l.getPos().y, l.getPos().x);
//       double angle = 
      
      
      ROS_INFO("legId: %d, peopleId: %d, confidence: %f, pos: (%f, %f), observations: %d, hasPair: %d, missed: %d, dist_traveled: %f, %s leg, conf_left_right: %f, angle: %f",
      l.getLegId(), l.getPeopleId(), confidence, l.getPos().x, l.getPos().y, l.getObservations(), l.hasPair(), l.getOccludedAge(), l.getDistanceTravelled(), left_or_right_leg.c_str(), conf_left_right, angle);
    }
  }
  
  void LegDetector::printPointCloudPoints(PointCloud& cloud, std::string name)
  {
    ROS_INFO("cloud %s: ", name.c_str());
    int i = 0;
    for (Point& p : cloud.points)
    {
      ROS_INFO("point: %d = (%f, %f)", i, p.x, p.y);
      i++;
    }
  }

  void LegDetector::removeOldMarkers(int from, int toExclusive, ros::Publisher ma_publisher)
  {
    visualization_msgs::MarkerArray ma;
    for (int i = from; i < toExclusive; i++) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = transform_link;
      marker.header.stamp = ros::Time();
      marker.ns = nh_.getNamespace();
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETE;
      ma.markers.push_back(marker);
    }
    ma_publisher.publish(ma);
  }
  

  void LegDetector::visLegs()
  {
    visualization_msgs::MarkerArray ma_leg;
    for (Leg& l : legs)
    {
      double pos_x = l.getPos().x;
      double pos_y = l.getPos().y;
      visualization_msgs::Marker m = getArrowMarker(pos_x, pos_y,
	       pos_x + 0.5 * l.getVel().x, pos_y + 0.5 * l.getVel().y, getNextLegsMarkerId());
      if (left_right.first == l.getLegId())
      {
	m.color.r = 0.;
	m.color.b = 1.0;
      }
      ma_leg.markers.push_back(m);
    }
    legs_and_vel_direction_publisher.publish(ma_leg);
  }

  unsigned int LegDetector::getNextLegsMarkerId()
  {
    return legs_marker_next_id++;
  }


  visualization_msgs::Marker LegDetector::getArrowMarker(double start_x, double start_y, double end_x, double end_y, int id)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start, end;
    start.x = start_x;
    start.y = start_y;
    start.z = z_coordinate / 2;
    end.x = end_x;
    end.y = end_y;
    end.z = z_coordinate / 2;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.1;
    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.r = 1.0;
    return marker;
  }

  // only for the user of the robot platform
  void LegDetector::matchClusterCentroidsToLegs(PointCloud cluster_centroids)
  {
    bool toReset = false;
    for (int i = 0; i < legs.size(); i++)
    {
      if (!legs[i].is_dead()) {
	
	if (legs.size() == 2)
	{
	  if (legs[i].getOccludedAge() < 3)
	  {
	    legs[i].predict();
	  }
	}
	else if (legs.size() == 1)
	{
	  legs[0].predict();
	}
	
	if (legs[i].getPos().x > x_upper_limit || legs[i].getPos().y > y_upper_limit || 
	  legs[i].getPos().x < x_lower_limit || legs[i].getPos().y < y_lower_limit 
	  )
	{
	  toReset = true;
	  break;
	}
      }
    }
    
    if (legs.size() == 2)
    {
      if (distanceBtwTwoPoints(legs[0].getPos(), legs[1].getPos()) > max_dist_btw_legs)
      {
	toReset = true;
      }
    }
    
    
    if (toReset) 
    {
      legs.clear();
      resetTrackingZone();
      resetLeftRight();
      return;
    }
    
    
    if (cluster_centroids.points.size() == 0) 
    { 
      for (int i = 0; i < legs.size(); i++)
      {
	legs[i].missed();
      }
      return; 
    }
    
    if (cluster_centroids.points.size() == 1) {
      Point p = cluster_centroids.points[0];
      double min_dist = max_cost;
      int index = -1;
      for (int i = 0; i < legs.size(); i++) {
	double cov = legs[i].getMeasToTrackMatchingCov();
	double mahalanobis_dist = std::sqrt((std::pow((p.x - legs[i].getPos().x), 2) +
	  std::pow((p.y - legs[i].getPos().y), 2)) / cov);
	double euclid_dist = distanceBtwTwoPoints(p, legs[i].getPos());
	if (legs.size() == 2)
	{
	  if ((i == 0 && distanceBtwTwoPoints(p, legs[1].getPos()) <= 0.05)
	    || (i == 1 && distanceBtwTwoPoints(p, legs[0].getPos()) <= 0.05))
	  {
	    continue;
	  }
	}
	if (mahalanobis_dist < min_dist && euclid_dist < 0.25)
	{
	  index = i;
	  min_dist = mahalanobis_dist;
	}
      }
      if (index != -1)
      {
	legs[index].update(p);
	
	// clear cloud 
	cluster_centroids.points.clear();
	
	if (legs.size() == 2)
	{
	  legs[1 - index].missed();
	}
      }
    } else if (legs.size() == 1) {
      double min_dist = max_cost;
      int index = -1;
      for (int i = 0; i < cluster_centroids.points.size(); i++) {
	double cov = legs[0].getMeasToTrackMatchingCov();
	double mahalanobis_dist = std::sqrt((std::pow((cluster_centroids.points[i].x - legs[0].getPos().x), 2) +
	  std::pow((cluster_centroids.points[i].y - legs[0].getPos().y), 2)) / cov);
	double euclid_dist = distanceBtwTwoPoints(cluster_centroids.points[i], legs[0].getPos());
	if (euclid_dist <= 0.05)
	{
	  index = i;
	  break;
	}
	if (mahalanobis_dist < min_dist && euclid_dist < 0.3)
	{
	  index = i;
	  min_dist = mahalanobis_dist;
	}
      }
      if (index != -1)  
      { 
	legs[0].update(cluster_centroids.points[index]);
	cluster_centroids.points.erase(cluster_centroids.points.begin() + index);
      }	
      else
      {
	legs[0].missed();
      }
    } else if (legs.size() == 2 && cluster_centroids.points.size() > 1) {
      
      double total_cost = max_cost;
      double fst_cov = legs[0].getMeasToTrackMatchingCov();
      double snd_cov = legs[1].getMeasToTrackMatchingCov();
      int fst_index = -1, snd_index = -1;
      int best_fst_index = -1, best_snd_index = -1;
      
      for (int i = 0; i < cluster_centroids.points.size(); i++) {
	double fst_euclid_dist = distanceBtwTwoPoints(cluster_centroids.points[i], legs[0].getPos());
// 	if (euclid_dist > 0.25 && cluster_centroids.points.size() ) { continue; }
	double fst_cost;
	if (fst_euclid_dist <= 0.05)
	{
	  fst_cost = 0.;
	  best_fst_index = i;
	}
	else
	{
	  fst_cost = std::sqrt((std::pow((cluster_centroids.points[i].x - legs[0].getPos().x), 2) +
	    std::pow((cluster_centroids.points[i].y - legs[0].getPos().y), 2)) / fst_cov);
	}
	for (int j = 0; j < cluster_centroids.points.size(); j++) {
	  if (i == j) { continue; }
	  double snd_euclid_dist = distanceBtwTwoPoints(cluster_centroids.points[j], legs[1].getPos());
	  double snd_cost;
	  
	  if (snd_euclid_dist <= 0.05)
	  {
	    snd_cost = 0.;
	    best_snd_index = j;
	  }
	  else
	  {
	    snd_cost = std::sqrt((std::pow((cluster_centroids.points[j].x - legs[1].getPos().x), 2) +
		std::pow((cluster_centroids.points[j].y - legs[1].getPos().y), 2)) / snd_cov);
	  }
	  
	  if (fst_cost + snd_cost < total_cost) {
	    total_cost = fst_cost + snd_cost;
	    fst_index = i;
	    snd_index = j;
	  }
	}
      }
      
      if (fst_index != -1)
      {
	legs[0].update(cluster_centroids.points[fst_index]);
      }
      else if (best_fst_index != -1)
      {
	legs[0].update(cluster_centroids.points[best_fst_index]);
      }
      else
      {
	legs[0].missed();
      }
      
      if (snd_index != -1)
      {
	legs[1].update(cluster_centroids.points[snd_index]);
      }
      else if (best_snd_index != -1)
      {
	legs[1].update(cluster_centroids.points[best_snd_index]);
      }
      else
      {
	legs[1].missed();
      }
    } 
    
    
    if (legs.size() == 2) {
      double uncertainty = 0.2;
      x_lower_limit_dynamic = std::min(legs[0].getPos().x, legs[1].getPos().x);
      x_lower_limit_dynamic -= uncertainty;
      
      x_upper_limit_dynamic = std::max(legs[0].getPos().x, legs[1].getPos().x);
      x_upper_limit_dynamic += uncertainty;
      
      y_lower_limit_dynamic = std::min(legs[0].getPos().y, legs[1].getPos().y);
      y_lower_limit_dynamic -= uncertainty;
      
      y_upper_limit_dynamic = std::max(legs[0].getPos().y, legs[1].getPos().y);
      y_upper_limit_dynamic += uncertainty;
      
      if (x_lower_limit_dynamic < x_lower_limit) {
	x_lower_limit_dynamic = x_lower_limit;
      }
      if (x_upper_limit_dynamic > x_upper_limit) {
	x_upper_limit_dynamic = x_upper_limit;
      }
      if (y_lower_limit_dynamic < y_lower_limit) {
	y_lower_limit_dynamic = y_lower_limit;
      }
      if (y_upper_limit_dynamic > y_upper_limit) {
	y_upper_limit_dynamic = y_upper_limit;
      }
      if ((x_upper_limit_dynamic - x_lower_limit_dynamic) * (y_upper_limit_dynamic - y_lower_limit_dynamic) < 0.17)
      {
	resetTrackingZone();
      }
    }

    if (legs.size() == 2) { return; }
    
    int used_point_index = -1;
    Point ref_point;
    ref_point.x = ref_point_x; 
    ref_point.y = ref_point_y;
    
    while (legs.size() < 2) {
      double min_dist = max_cost;
      int index = -1;
      for (int i = 0; i < cluster_centroids.points.size(); i++)
      {
	if (i == used_point_index) { continue; }
	double dist = distanceBtwTwoPoints(ref_point, cluster_centroids.points[i]);
	if (dist < min_dist && dist < 0.3)
	{
	  index = i;
	  min_dist = dist;
	}
      }
      if (index != -1) 
      {
	used_point_index = index;
	legs.push_back(initLeg(cluster_centroids.points[index]));
      } 
      else
      {
	// there is no valid point in cloud
	break;
      }
      
      // cloud has only one point
      if (used_point_index != -1 && cluster_centroids.points.size() <= 1) { break; }
    }
  }
  
  
  void LegDetector::resetTrackingZone() 
  {
    x_lower_limit_dynamic = x_lower_limit;
    x_upper_limit_dynamic = x_upper_limit;
    y_lower_limit_dynamic = y_lower_limit;
    y_upper_limit_dynamic = y_upper_limit;
  }
  
  
  double LegDetector::how_much_in_free_space(double x, double y)
  {
    // Determine the degree to which the position (x,y) is in freespace according to our global map
    if (!got_map) {
      return in_free_space_threshold * 2;
    }
    int map_x = int(std::round((x - global_map.info.origin.position.x)/global_map.info.resolution));
    int map_y = int(std::round((y - global_map.info.origin.position.y)/global_map.info.resolution));
    
    double sum = 0;
    int kernel_size = 2;
    for (int i = map_x - kernel_size; i <= map_x + kernel_size; i++) {  
      for (int j = map_y - kernel_size; j <= map_y + kernel_size; j++) { 
	  if (i + j * global_map.info.height < global_map.info.height * global_map.info.width) {
	      sum += global_map.data[i + j * global_map.info.height];
	  } else {  
	      // We went off the map! position must be really close to an edge of global_map
	      return in_free_space_threshold * 2;
	  }
      }
    }
    double percent = sum / (( std::pow((2. * kernel_size + 1.), 2)) * 100.);
    return percent;
  }


  void LegDetector::separateLegs(int i, int j)
  {
    int pId = legs[i].getPeopleId();
    legs[i].setHasPair(false);
    legs[j].setHasPair(false);
    legs[i].setPeopleId(-1);
    legs[j].setPeopleId(-1);
    
    if (isBoundingBoxTracking)
    {
      for (int i = 0; i < tracking_zones.size(); i++)
      {
	if (tracking_zones[i].getPeopleId() == pId)
	{
	  tracking_zones.erase(tracking_zones.begin() + i);
	  break;
	}
      }
    }
  }

  
  void LegDetector::checkDistanceOfLegs()
  {
    for (int i = 0; i < legs.size(); i++)
    {
      if (!legs[i].hasPair()) { continue; }
      for (int j = i + 1; j < legs.size(); j++)
      {
	if (legs[i].getPeopleId() == legs[j].getPeopleId()) {
	  if (distanceBtwTwoPoints(legs[i].getPos(), legs[j].getPos()) > max_dist_btw_legs)
	  {
	    separateLegs(i, j);
	  }
	  break;
	}
      }
    }
  }
  

  void LegDetector::findPeople()
  {
    checkDistanceOfLegs();
    for (int i = 0; i < legs.size(); i++)
    {
      if (legs[i].hasPair() || (legs[i].getPeopleId() == -1 && legs[i].getObservations() < min_observations)) 
	{ continue; }
      findSecondLeg(i);
    }
  }

  void LegDetector::findSecondLeg(int fst_leg)
  {
    std::vector<int> indices_of_potential_legs;
    for (int i = fst_leg + 1; i < legs.size(); i++)
    {
      if (legs[i].hasPair() || legs[i].getObservations() < min_observations
	|| distanceBtwTwoPoints(legs[fst_leg].getPos(), legs[i].getPos()) > max_dist_btw_legs
	|| distanceBtwTwoPoints(legs[fst_leg].getPos(), legs[i].getPos()) < leg_radius)
      { continue; }
      indices_of_potential_legs.push_back(i);
    }

    if (indices_of_potential_legs.size() == 0) { ROS_DEBUG("There is no potential second leg!"); return; }

    if (indices_of_potential_legs.size() == 1) {
      setPeopleId(fst_leg, indices_of_potential_legs[0]);
      return;
    }
    int snd_leg = -1;
    
    double max_gain = 0.;
    for (int i = 0; i < indices_of_potential_legs.size(); i++)
    {
      double gain = 0.;
      bool isHistoryDistanceValid = true;
      int history_size = legs[fst_leg].getHistory().size();
      if (history_size != min_observations || 
	legs[indices_of_potential_legs[i]].getHistory().size() != history_size)
      {
        ROS_WARN("History check: vectors are not equal in size!");
        return;
      }
      for (int j = 0; j < history_size - 1; j++)
      {
	int fst_history_size = legs[fst_leg].getHistory()[j].size();
	int snd_history_size = legs[indices_of_potential_legs[i]].getHistory()[j].size();
	
	if (fst_history_size != state_dimensions || snd_history_size != state_dimensions)
	{
	  ROS_WARN("History check: stae vectors are not valid!");
	  return;
	}
	
	double dist = distanceBtwTwoPoints(legs[fst_leg].getHistory()[j][0], 
					   legs[fst_leg].getHistory()[j][1],
	  legs[indices_of_potential_legs[i]].getHistory()[j][0], 
	  legs[indices_of_potential_legs[i]].getHistory()[j][1]);
	
	if (dist > max_dist_btw_legs)
	{
	  ROS_DEBUG("History check: distance is not valid!");
	  isHistoryDistanceValid = false;
	  break;
	}
	
        double forgettingFactor = std::pow(0.5, history_size - 1 - j);
        gain += forgettingFactor * (1 - dist / std::sqrt(200));
      }

      gain /= history_size;
      
      if (!isHistoryDistanceValid) { continue; }
      if (max_gain < gain) {
        max_gain = gain;
        snd_leg = indices_of_potential_legs[i];
      }
    }

    if (snd_leg == -1) { ROS_DEBUG("Could not find second leg!"); return; }

    setPeopleId(fst_leg, snd_leg);
  }
  

  double LegDetector::distanceBtwTwoPoints(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
  }

  
  double LegDetector::distanceBtwTwoPoints(Point p1, Point p2)
  {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
  }


  void LegDetector::setPeopleId(int fst_leg, int snd_leg)
  {
    int id = -1;
    bool isIdSet = false;
    
    bool restoreId = false;
    Point peoplePos;
    peoplePos.x = (legs[fst_leg].getPos().x + legs[snd_leg].getPos().x) / 2;
    peoplePos.y = (legs[fst_leg].getPos().y + legs[snd_leg].getPos().y) / 2;
    
    for (int i = 0; i < lastSeenPeoplePositions.size(); i++) 
    {
      Point lastPos = std::get<2>(lastSeenPeoplePositions[i]);
      if (distanceBtwTwoPoints(peoplePos, lastPos) > max_dist_btw_legs) { continue; }
      id = std::get<1>(lastSeenPeoplePositions[i]);
      
      restoreId = true;
      removeLastSeenPeoplePosition(i);
      break;
    }
    
    if (!restoreId || id == -1) 
    {
      isIdSet = legs[fst_leg].getPeopleId() != -1 || legs[snd_leg].getPeopleId() != -1;
      if (isIdSet && (legs[snd_leg].getPeopleId() ==  -1 || legs[snd_leg].getPeopleId() == legs[fst_leg].getPeopleId()) )
      {
	id = legs[fst_leg].getPeopleId();
	eraseRemovedLeg(id);
      }
      else if (isIdSet && legs[fst_leg].getPeopleId() ==  -1)
      {
	id = legs[snd_leg].getPeopleId();
	eraseRemovedLeg(id);
      }
      else if (isIdSet && legs[snd_leg].getPeopleId() != legs[fst_leg].getPeopleId())
      {
	id = std::min(legs[fst_leg].getPeopleId(), legs[snd_leg].getPeopleId());
	eraseRemovedLeg(legs[fst_leg].getPeopleId());
	eraseRemovedLeg(legs[snd_leg].getPeopleId());
      }
      else
      {
	id = id_counter++;
      }
    }
    
    legs[fst_leg].setPeopleId(id);
    legs[snd_leg].setPeopleId(id);
    legs[fst_leg].setHasPair(true);
    legs[snd_leg].setHasPair(true);
    
    if (isBoundingBoxTracking) 
    {
      BoundingBox b(legs[fst_leg].getLegId(), legs[snd_leg].getLegId(), id, legs[fst_leg].getPos().x, legs[snd_leg].getPos().x, legs[fst_leg].getPos().y, legs[snd_leg].getPos().y);
      tracking_zones.push_back(b);
    }
  }

  void LegDetector::eraseRemovedLegsWithoutId()
  {
    for (std::vector<Leg>::iterator it = removed_legs.begin(); it != removed_legs.end(); it++)
    {
      if (it->getPeopleId() == -1)
      {
	removed_legs.erase(it);
      }
    }
  }

  void LegDetector::eraseRemovedLeg(int id)
  {
    for (std::vector<Leg>::iterator it = removed_legs.begin(); it != removed_legs.end(); it++)
    {
      if (it->getPeopleId() == id)
      {
	removed_legs.erase(it);
	return;
      }
    }
  }
  
  
  visualization_msgs::Marker LegDetector::getOvalMarker(int id, double x, double y, 
      double orientation_x, double orientation_y, double orientation_z, double orientation_w,
      double scale_x, double scale_y, double r, double g, double b)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z_coordinate;
    marker.pose.orientation.x = orientation_x;
    marker.pose.orientation.y = orientation_y;
    marker.pose.orientation.z = orientation_z;
    marker.pose.orientation.w = orientation_w;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
  }
  

  visualization_msgs::Marker LegDetector::getOvalMarkerForTwoPoints(int pId, double x1, double y1, double x2, double y2, int id)
  {
    double x = (x1 + x2) / 2;
    double y = (y1 + y2) / 2;
    double dist = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    double scale_x = dist + 5 * leg_radius;
    double scale_y = 5 * leg_radius;
    double norm_1 = std::sqrt(std::pow(x1, 2) + std::pow(y1, 2));
    double norm_2 = std::sqrt(std::pow(x2, 2) + std::pow(y2, 2));
    double temp = (x1 * x2 + y1 * y2) / (norm_1 * norm_2);
    double diff_x = x1 - x2;
    double diff_y = y1 - y2;
    double angle = std::atan2( diff_y, diff_x );

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    double orientation_w = q.w();
    double orientation_x = q.x();
    double orientation_y = q.y();
    double orientation_z = q.z();
    
    double r = 0.0, g = 0.0, b = 0.0;
    if (pId != -1)
    {
      std::map<int, visualization_msgs::Marker>::iterator it = paths.find(pId);
      if (it != paths.end())
      {
	r = it->second.color.r;
	g = it->second.color.g;
	b = it->second.color.b;
      }
    }
    
    return getOvalMarker(id, x, y, orientation_x, orientation_y, orientation_z, orientation_w,
      scale_x, scale_y, r, g, b);
  }
  
  
  void LegDetector::checkIfLeftOrRight(int i, int j)
  {
    if (legs[i].getPos().y <= legs[j].getPos().y)
    {
      // leg i is the right leg 
      ij_or_ji.second++;
    }
    else
    {
      ij_or_ji.first++;
    }
      // if velocity of one leg is greater than 0.2 m/s and velocity of another leg is smaller than 0.2 m/s
    double fst_vel = calculateNorm(legs[i].getVel());
    double snd_vel = calculateNorm(legs[j].getVel());
    int moving_leg = -1, unmoving_leg = -1;

    if (fst_vel > 0.2 && snd_vel < 0.2)
    {
      moving_leg = i; 
      unmoving_leg = j;
    } 
    else if (fst_vel < 0.2 && snd_vel > 0.2)
    {
      moving_leg = j;
      unmoving_leg = i;
    }
    
    if (left_right.first == -1)
    {
      if (legs[i].getPos().y <= legs[j].getPos().y)
      {
	// leg i is the right leg 
	left_right = std::make_pair(legs[j].getLegId(), legs[i].getLegId());
      }
      else
      {
	left_right = std::make_pair(legs[i].getLegId(), legs[j].getLegId());
      }
    }
    else if (moving_leg != -1 && unmoving_leg != -1)
    {
      
      double Ax = legs[moving_leg].getPos().x;
      double Ay = legs[moving_leg].getPos().y;
      double Bx = legs[moving_leg].getPos().x + legs[moving_leg].getVel().x;
      double By = legs[moving_leg].getPos().y + legs[moving_leg].getVel().y;
      
      double dist_to_pos = distanceBtwTwoPoints(0., 0., Ax, Ay);
      double dist_to_vel = distanceBtwTwoPoints(0., 0., Bx, By);
      
      if (dist_to_pos > dist_to_vel)
      {
	// moving leg is the right leg
	if (legs[moving_leg].getPos().y <= legs[unmoving_leg].getPos().y)
	{
	  // check if this "right" leg is temporary on the right side
	  if ((moving_leg < unmoving_leg && ij_or_ji.first < ij_or_ji.second) || 
	    (moving_leg > unmoving_leg && ij_or_ji.first > ij_or_ji.second))
	  {
	    if (left_right.first != legs[unmoving_leg].getLegId())
	    {
	      left_right = std::make_pair(legs[unmoving_leg].getLegId(), legs[moving_leg].getLegId());
	      conf_left_right = 0.1;
	    }
	    else
	    {
	      conf_left_right = std::min(1., conf_left_right + 0.1);
	    }
	  }
	}
	else
	{
	  // moving leg is now on the left side -> if moving_leg < unmoving_leg then i else j
	  if ((moving_leg < unmoving_leg && ij_or_ji.first > ij_or_ji.second) || 
	    (moving_leg > unmoving_leg && ij_or_ji.first < ij_or_ji.second))
	  {
	    if (left_right.first != legs[moving_leg].getLegId())
	    {
	      left_right = std::make_pair(legs[moving_leg].getLegId(), legs[unmoving_leg].getLegId());
	      conf_left_right = 0.1;
	    }
	    else
	    {
	      conf_left_right = std::min(1., conf_left_right + 0.1);
	    }
	  }
	}
      }
    }
  }

  void LegDetector::vis_people(std_msgs::Header header)
  {
    visualization_msgs::MarkerArray ma_people;

    for (int i = 0; i < legs.size(); i++)
    {
      int id = legs[i].getPeopleId();
      if (id == -1) { continue; }

      	// second leg is removed
        if (!legs[i].hasPair())
        {
        for (int j = 0; j < removed_legs.size(); j++)
        {
            if (legs[j].getPeopleId() == id)
            {
            if (distanceBtwTwoPoints(legs[i].getPos(), legs[j].getPos()) > max_dist_btw_legs) {
                break;
            }
            updatePath(id, header, 
                    legs[i].getPos().x,
                    legs[i].getPos().y,
                    legs[j].getPos().x, 
                    legs[j].getPos().y);
            ma_people.markers.push_back(getOvalMarkerForTwoPoints(id,
                    legs[i].getPos().x,
                    legs[i].getPos().y,
                    legs[j].getPos().x, 
                    legs[j].getPos().y, 
                    getPeopleMarkerNextId()));
            publish_person_msg_stamped(i, j, header);
            break;
            }
        }

        } 
	else 
	{
	  for (int j = i + 1; j < legs.size(); j++)
	  {
	    if (legs[j].getPeopleId() == id)
	    {
	      updatePath(id, header, legs[i].getPos().x,
		  legs[i].getPos().y, legs[j].getPos().x, legs[j].getPos().y);
	      ma_people.markers.push_back(getOvalMarkerForTwoPoints(id, legs[i].getPos().x,
		  legs[i].getPos().y, legs[j].getPos().x, legs[j].getPos().y, getPeopleMarkerNextId()));
          publish_person_msg_stamped(i, j, header);
	      
	     checkIfLeftOrRight(i, j);
	     
	      break;
	    }
	  }
      }
    }

    people_pub.publish(ma_people);
  }
  
  void LegDetector::pubExtendedLine(double x1, double y1, double x2, double y2, int id)
  {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = transform_link;
    line_strip.header.stamp = ros::Time();
    line_strip.ns = nh_.getNamespace();
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id;
    line_strip.scale.x = 0.03;
    line_strip.color.a = 1.0;
    line_strip.color.r = getRandomNumberFrom0To1();
    line_strip.color.g = getRandomNumberFrom0To1();
    line_strip.color.b = getRandomNumberFrom0To1();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    geometry_msgs::Point p;
    p.x = x1;
    p.y = y1;
    line_strip.points.push_back(p);
    p.x = 3 * x2 - x1;
    p.y = 3 * y2 - y1;
    line_strip.points.push_back(p);
    marker_pub.publish(line_strip);
  }
  
  void LegDetector::updatePath(unsigned int pId, std_msgs::Header header, double x1, double y1, double x2, double y2)
  {
    geometry_msgs::Point p;
    p.x = (x1 + x2) / 2;
    p.y = (y1 + y2) / 2;
    std::map<int, visualization_msgs::Marker>::iterator it = paths.find(pId);
    if (it != paths.end())
    {
      it->second.points.push_back(p);
    }
    else
    {
      visualization_msgs::Marker line_strip;
      line_strip.header = header;
      line_strip.ns = nh_.getNamespace();
      line_strip.action = visualization_msgs::Marker::ADD;
      line_strip.pose.orientation.w = 1.0;
      line_strip.id = pId;
      line_strip.scale.x = 0.03;
      line_strip.color.a = 1.0;
      line_strip.color.r = getRandomNumberFrom0To1();
      line_strip.color.g = getRandomNumberFrom0To1();
      line_strip.color.b = getRandomNumberFrom0To1();
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      line_strip.points.push_back(p);
      paths.insert(std::make_pair(pId, line_strip));
    }
  }


  unsigned int LegDetector::getPeopleMarkerNextId() {
    return people_marker_next_id++;
  }


  bool LegDetector::clustering(const PointCloud& cloud, PointCloud& cluster_centroids)
  {
    if (cloud.points.size() < minClusterSize) { ROS_DEBUG("Clustering: Too small number of points!"); return false; }

    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud (cloud.makeShared());
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud.makeShared());
    ec.extract(cluster_indices);

    cluster_centroids.header = cloud.header;
    cluster_centroids.points.clear();
    
    PointCloud cluster_centroids_temp, leg_positions;
    cluster_centroids_temp.header = cloud.header;
    leg_positions.header = cloud.header;
    
    std::vector<std::pair<Point, Point> > minMaxPoints;
    
    for (Leg& l : legs) 
    {
      if (l.getPeopleId() != -1)
      {
	leg_positions.points.push_back(l.getPos());
      }
    }
    pcl::KdTreeFLANN<Point> kdtree_clusters, kdtree_legs;
    
    if (leg_positions.points.size() != 0) {
      kdtree_legs.setInputCloud(leg_positions.makeShared());
    }

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<Point>::Ptr cloud_cluster (new pcl::PointCloud<Point>);
      cloud_cluster->header = cloud.header;
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	cloud_cluster->points.push_back (cloud.points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      
      if (cloud.points.size() > 2)
      {
	pubExtendedLine(0., 0., cloud.points[0].x, cloud.points[0].y, 0);
	pubExtendedLine(0., 0., cloud.points[cloud.points.size() - 1].x, cloud.points[cloud.points.size() - 1].y, 1);
      }
      
      Point min, max;
      pcl::getMinMax3D(*cloud_cluster, min, max);
      min.x -= cluster_bounding_box_uncertainty;
      min.y -= cluster_bounding_box_uncertainty; 
      max.x += cluster_bounding_box_uncertainty; 
      max.y += cluster_bounding_box_uncertainty; 
      minMaxPoints.push_back(std::make_pair(min, max));
      
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud_cluster, centroid);
      
      Point p; p.x = centroid(0); p.y = centroid(1);
      
      
      if (leg_positions.points.size() != 0) {
	std::vector<int> pointIdxRadius;
	std::vector<float> pointsSquaredDistRadius;
	// radius search
	int count = kdtree_legs.radiusSearch(p, 0.03, pointIdxRadius, 
							  pointsSquaredDistRadius);
	if (count == 1) {
	  cluster_centroids_temp.points.push_back(leg_positions.points[pointIdxRadius[0]]);
	} else {
	  cluster_centroids_temp.points.push_back(p);
	}
      } else {
	cluster_centroids.points.push_back(p);
      }
    }
    
    if (cluster_centroids_temp.points.size() == 0) { return true; }
    
    kdtree_clusters.setInputCloud(cluster_centroids_temp.makeShared());
    
    double radius = 0.2;
    
    std::map<int, bool> map_removed_indices;
    
    for (int i = 0; i < cluster_centroids_temp.points.size(); i++)
    {
      std::map<int, bool>::iterator removed_indices_it = map_removed_indices.find(i);
      if (removed_indices_it != map_removed_indices.end()) { continue; }
      
      Point cluster = cluster_centroids_temp.points[i];
      std::vector<int> pointIdxRadius_clusters, pointIdxRadius_legs;
      std::vector<float> pointsSquaredDistRadius_clusters, pointsSquaredDistRadius_legs;
      
      int K = 2;
      std::vector<int> pointsIdx(K);
      std::vector<float> pointsSquaredDist(K);
      
//       radius search
      int count_clusters = kdtree_clusters.radiusSearch(cluster, radius, pointIdxRadius_clusters, 
							pointsSquaredDistRadius_clusters);
      
      int count_legs = kdtree_legs.nearestKSearch(cluster, K, pointsIdx, pointsSquaredDist);
      
      if (pointsIdx.size() != K) { 
	cluster_centroids.points.push_back(cluster); 
	continue; 
      }
      
      Point fst_leg, snd_leg;
      fst_leg = leg_positions[pointsIdx[0]];
      snd_leg = leg_positions[pointsIdx[1]];
      bool isFstPointInBox = (fst_leg.x >= minMaxPoints[i].first.x) && (fst_leg.y >= minMaxPoints[i].first.y)
	&& (fst_leg.x <= minMaxPoints[i].second.x) && (fst_leg.y <= minMaxPoints[i].second.y);
      bool isSndPointInBox = (snd_leg.x >= minMaxPoints[i].first.x) && (snd_leg.y >= minMaxPoints[i].first.y)
	&& (snd_leg.x <= minMaxPoints[i].second.x) && (snd_leg.y <= minMaxPoints[i].second.y);
      
      if (isFstPointInBox && isSndPointInBox)
      { 
	
	std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
	it += i;
	
	PointCloud fst, snd;
	fst.header = cloud.header;
	snd.header = cloud.header;
	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	{ 
	  Point p = cloud.points[*pit];
	  double dot_product = p.x * cluster.y - cluster.x * p.y;
	  if (dot_product < 0) { fst.points.push_back(p); }
	  else { snd.points.push_back(p); }
	}
	
	if (fst.points.size() < minClusterSize || snd.points.size() < minClusterSize) 
	{
	  cluster_centroids.points.push_back(cluster);
	  continue;
	}
	
	Eigen::Vector4f centroid_fst, centroid_snd;
	pcl::compute3DCentroid(fst, centroid_fst);
	pcl::compute3DCentroid(snd, centroid_snd);
	
	Point p_fst, p_snd;
	p_fst.x = centroid_fst(0); p_fst.y = centroid_fst(1);
	p_snd.x = centroid_snd(0); p_snd.y = centroid_snd(1);
	
	
	if (distanceBtwTwoPoints(p_fst, p_snd) < leg_radius) 
	{
	  cluster_centroids.points.push_back(cluster);
	  continue;
	}
	
	cluster_centroids.points.push_back(p_fst);
	cluster_centroids.points.push_back(p_snd);
      }
      else
      {
	cluster_centroids.points.push_back(cluster);
      }
    }
    return true;
  }
  
  
  
  void LegDetector::pub_bounding_box(double min_x, double min_y, double max_x, double max_y)
  {	
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (max_x + min_x) / 2;
    marker.pose.position.y = (max_y + min_y) / 2;
    marker.pose.position.z = z_coordinate;
    marker.scale.x = max_x - min_x;
    marker.scale.y = max_y - min_y;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
//     bounding_box_pub.publish(marker);
  }
  
  void LegDetector::pub_border(double min_x, double min_y, double max_x, double max_y)
  {	
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (max_x + min_x) / 2;
    marker.pose.position.y = (max_y + min_y) / 2;
    marker.pose.position.z = z_coordinate;
    marker.scale.x = max_x - min_x;
    marker.scale.y = max_y - min_y;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    tracking_area_pub.publish( marker );
  }
  

  void LegDetector::predictLegs()
  {
    for (int i = 0; i < legs.size(); i++) {
      legs[i].predict(); 
      legs[i].missed();
    }
    if (!isOnePersonToTrack)
    {
      cullDeadTracks(legs);
    }
  }

  void LegDetector::removeLegFromVector(std::vector<Leg>& v, unsigned int i)
  {
    // save people id at v[i] 
    if (v[i].getPeopleId() != -1 && !v[i].hasPair()) 
    {
      for (int j = 0; j < removed_legs.size(); j++) 
      {
	if (removed_legs[j].getPeopleId() != v[i].getPeopleId()) { continue; }
	
	Point peoplePos;
	peoplePos.x = (v[i].getPos().x + removed_legs[j].getPos().x) / 2;
	peoplePos.y = (v[i].getPos().y + removed_legs[j].getPos().y) / 2;
	
	lastSeenPeoplePositions.push_back(std::make_tuple(0, v[i].getPeopleId(), peoplePos));
      }
    }
      
    if (i != v.size() - 1) {
      Leg l = v[v.size() - 1];
      v[i] = l;
    }
    v.pop_back();
  }

  void LegDetector::resetHasPair(std::vector<Leg>& v, int fst_leg)
  {
    for (int snd_leg = 0; snd_leg < v.size(); snd_leg++) {
      if (snd_leg != fst_leg && v[snd_leg].getPeopleId() == v[fst_leg].getPeopleId()) {
	v[snd_leg].setHasPair(false);
	break;
      }
    }
  }

  void LegDetector::cullDeadTracks(std::vector<Leg>& v)
  {
    int i = 0;
    while(i < v.size()) {
      if (v[i].is_dead()) {
	if (v[i].hasPair()) { resetHasPair(v, i); removed_legs.push_back(v[i]); }
	removeLegFromVector(v, i);
      } else {
	i++;
      }
    }			
  }

  unsigned int LegDetector::getNextCovEllipseId()
  {
    return cov_ellipse_id++;
  }
  
  void LegDetector::boundingBoxTracking(PointCloud& cluster_centroids)
  {
    std::map<int, int> id_to_index_map;
    
    for (int i = 0; i < legs.size(); i++)
    {
      if (!legs[i].is_dead()) {
	legs[i].predict();
      }
      id_to_index_map.insert(std::make_pair(legs[i].getLegId(), i));
    }
    
    
    std::map<int, int>::iterator it;
    for (int j = 0; j < tracking_zones.size(); j++) 
    {
      int fst_leg = -1, snd_leg = -1;
      it = id_to_index_map.find(tracking_zones[j].getFstLegId());
      if (it != id_to_index_map.end()) { fst_leg = it->second; }
      else { ROS_ERROR("Could not find index for leg id!"); continue; }
      it = id_to_index_map.find(tracking_zones[j].getSndLegId());
      if (it != id_to_index_map.end()) { snd_leg = it->second; }
      else { ROS_ERROR("Could not find index for leg id!"); continue; }
      if (fst_leg == -1 || snd_leg == -1) { continue; }
      
      
      tracking_zones[j].update(legs[fst_leg].getPos().x, legs[fst_leg].getPos().y, legs[snd_leg].getPos().x, legs[snd_leg].getPos().y);
    }
    
    if (cluster_centroids.points.size() == 0) { return; }
    
    std::map<int, bool> usedPointsIds;
    
    for (int i = 0; i < tracking_zones.size(); i++)
    {
      PointCloud used_points_cloud;
      used_points_cloud.header = cluster_centroids.header;
      std::map<int, bool>::iterator it;
      for (int j = 0; j < cluster_centroids.points.size(); j++) 
      {
	it = usedPointsIds.find(j);
	if (it != usedPointsIds.end()) { continue; } // "it != end" means that the point is used
	if (cluster_centroids.points[j].x >= tracking_zones[i].getXLowerLimit() &&
	    cluster_centroids.points[j].x <= tracking_zones[i].getXUpperLimit() &&
	    cluster_centroids.points[j].y >= tracking_zones[i].getYLowerLimit() &&
	    cluster_centroids.points[j].y <= tracking_zones[i].getYUpperLimit())
	{
	  usedPointsIds.insert(std::make_pair(j, true));
	  used_points_cloud.points.push_back(cluster_centroids.points[j]);
	}
      }
      matchClusterCentroids2Legs(tracking_zones[i].getFstLegId(), tracking_zones[i].getSndLegId(), used_points_cloud, i);
    }
    
    PointCloud rest_points;
    rest_points.header = cluster_centroids.header;
    for (int j = 0; j < cluster_centroids.points.size(); j++) 
    {
      std::map<int, bool>::iterator it;
      it = usedPointsIds.find(j);
      if (it == usedPointsIds.end()) // j is not in the map
      { 
	rest_points.points.push_back(cluster_centroids.points[j]);
      }
    }
    
    if (rest_points.points.size() == 0) { return; }
    
    std::vector<Leg> legsWithoudPeopleId, legsWithPeopleId, fused;
    
    for (int i = 0; i < legs.size(); i++) 
    {
      if (legs[i].getPeopleId() != -1) 
      { 
	legsWithPeopleId.push_back(legs[i]);
      }
      else
      {
	legsWithoudPeopleId.push_back(legs[i]);
      }
    }
    
    assign_munkres(rest_points, legsWithoudPeopleId, fused);
    
    cullDeadTracks(fused);
    
    for (int i = 0; i < fused.size(); i++) 
    {
      legsWithPeopleId.push_back(fused[i]);
    }
    
    legs = legsWithPeopleId;
  }
  
  void LegDetector::matchClusterCentroids2Legs(unsigned int fst_leg_id, unsigned int snd_leg_id, PointCloud& cluster_centroids, int tracking_zone_index)
  {
    int fst_leg = -1, snd_leg = -1;
    for (int i = 0; i < legs.size(); i++) {
      if (legs[i].getLegId() == fst_leg_id) { fst_leg = i; }
      if (legs[i].getLegId() == snd_leg_id) { snd_leg = i; }
    }
    if (fst_leg == -1 || snd_leg == -1 || cluster_centroids.points.size() == 0) 
    { 
      return; 
    }
    
    if (cluster_centroids.points.size() == 1) 
    {
      Point p = cluster_centroids.points[0];
      
      double fst_cov = legs[fst_leg].getMeasToTrackMatchingCov();
      double snd_cov = legs[snd_leg].getMeasToTrackMatchingCov();
	     
      double fst_mahalanobis_dist = std::sqrt((std::pow((p.x - legs[fst_leg].getPos().x), 2) +
	  std::pow((p.y - legs[fst_leg].getPos().y), 2)) / fst_cov);
      double snd_mahalanobis_dist = std::sqrt((std::pow((p.x - legs[snd_leg].getPos().x), 2) +
	  std::pow((p.y - legs[snd_leg].getPos().y), 2)) / snd_cov);
      
      double fst_euclidian_dist = distanceBtwTwoPoints(p, legs[fst_leg].getPos());
      double snd_euclidian_dist = distanceBtwTwoPoints(p, legs[snd_leg].getPos());
      
      if (fst_euclidian_dist > 0.3 && snd_euclidian_dist > 0.3) { return; }
      
      if (fst_mahalanobis_dist <= snd_mahalanobis_dist) 
      {
	legs[fst_leg].update(p);
      }
      else
      {
	legs[snd_leg].update(p);
      }
    } 
    else 
    {
      double total_cost = max_cost;
      double fst_cov = legs[fst_leg].getMeasToTrackMatchingCov();
      double snd_cov = legs[snd_leg].getMeasToTrackMatchingCov();
      int fst_index = -1, snd_index = -1;
      
      for (int i = 0; i < cluster_centroids.points.size(); i++) 
      {
	double fst_cost = std::sqrt((std::pow((cluster_centroids.points[i].x - legs[fst_leg].getPos().x), 2) +
	  std::pow((cluster_centroids.points[i].y - legs[fst_leg].getPos().y), 2)) / fst_cov);
	for (int j = 0; j < cluster_centroids.points.size(); j++) 
	{
	  if (i == j) { continue; }
	  double snd_cost = std::sqrt((std::pow((cluster_centroids.points[j].x - legs[i].getPos().x), 2) +
	    std::pow((cluster_centroids.points[j].y - legs[i].getPos().y), 2)) / snd_cov);
	  if (fst_cost + snd_cost < total_cost) 
	  {
	    total_cost = fst_cost + snd_cost;
	    fst_index = i;
	    snd_index = j;
	  }
	}
      }
      
      if (fst_index == -1 || snd_index == -1) { return; }
      
      double euclid_dist = distanceBtwTwoPoints(cluster_centroids.points[fst_index], legs[fst_leg].getPos());
      if (euclid_dist < 0.33) 
      {
	legs[fst_leg].update(cluster_centroids.points[fst_index]);
      }
      
      legs[snd_leg].update(cluster_centroids.points[snd_index]);
      euclid_dist = distanceBtwTwoPoints(cluster_centroids.points[snd_index], legs[snd_leg].getPos());
      if (euclid_dist < 0.33) 
      {
	legs[snd_leg].update(cluster_centroids.points[snd_index]);
      }
    } 
    
    //udpate tracking zone
    tracking_zones[tracking_zone_index].update(legs[fst_leg].getPos().x, legs[fst_leg].getPos().y, legs[snd_leg].getPos().x, legs[snd_leg].getPos().y);
  }
  

  void LegDetector::gnn_munkres(PointCloud& cluster_centroids)
  {
    // Filter model predictions
    for (int i = 0; i < legs.size(); i++)
    {	
      if (legs[i].hasPair()) {
	double vel = calculateNorm(legs[i].getVel());
	if (vel > 0.2) {
	  for (int j = i + 1; j < legs.size(); j++) {
	    if (legs[i].getPeopleId() == legs[j].getPeopleId()) {
	      double dist = distanceBtwTwoPoints(legs[i].getPos(), legs[j].getPos());
	      if (max_dist_btw_legs - dist < 0.1) {
		legs[i].resetErrorCovAndState();
	      }
	      break;
	    }
	  }
	}
      }
      
      legs[i].predict();
    }
    
    if (cluster_centroids.points.size() == 0) { return; }
    
    std::vector<Leg> fused;

    assign_munkres(cluster_centroids, legs, fused);

    cullDeadTracks(fused);
    
    legs = fused;
  }

  
  void LegDetector::assign_munkres(const PointCloud& meas,
		    std::vector<Leg> &tracks,
		    std::vector<Leg> &fused)
  {
      // Create cost matrix between previous and current blob centroids
      int meas_count = meas.points.size();
      int tracks_count = tracks.size();

      // Determine max of meas_count and tracks
      int rows = -1, cols = -1;
      if (meas_count >= tracks_count) {
	  rows = cols = meas_count;
      } else {
	  rows = cols = tracks_count;
      }
      
      Matrix<double> matrix(rows, cols);
      
      visualization_msgs::MarkerArray cov_ellipse_ma;
      if (cov_ellipse_id != 0) {
	removeOldMarkers(0, cov_ellipse_id, cov_marker_pub);
	cov_ellipse_id = 0;
      }

      // New measurements are along the Y-axis (left hand side)
      // Previous tracks are along x-axis (top-side)
      int r = 0;
      for(const Point& p : meas.points) {
	std::vector<Leg>::iterator it_prev = tracks.begin();
	int c = 0;
	for (; it_prev != tracks.end(); it_prev++, c++) {
	  Eigen::MatrixXd cov_matrix = it_prev->getMeasToTrackMatchingCovMatrix();
	  double cov = it_prev->getMeasToTrackMatchingCov();
	  if (cov == 0) { ROS_ERROR("assign_munkres: cov = 0"); continue; }
	  double mahalanobis_dist = std::sqrt((std::pow((p.x - it_prev->getPos().x), 2) +
						std::pow((p.y - it_prev->getPos().y), 2)) / cov);
	  double dist = distanceBtwTwoPoints(p, it_prev->getPos());
// 	  cov_ellipse_ma.markers.push_back(getCovarianceEllipse(getNextCovEllipseId(), it_prev->getPos().x,
// 	    it_prev->getPos().y, cov_matrix));
	  
	  if (dist <= 0.03)
	  {
	    matrix(r, c) = 0;
	  } 
	  else if (mahalanobis_dist < mahalanobis_dist_gate && dist < 0.6) 
	  {
	    matrix(r, c) = mahalanobis_dist;
	  } 
	  else 
	  {
	    matrix(r, c) = max_cost;
	  }
	}
	r++;
      }
      
//       cov_marker_pub.publish(cov_ellipse_ma);
      
      Munkres<double> m;
      m.solve(matrix);
      
      // Use the assignment to update the old tracks with new blob measurement
      int meas_it = 0;
      for(r = 0; r < rows; r++) {
	std::vector<Leg>::iterator it_prev = tracks.begin();
	for (int c = 0; c < cols; c++) {
	    if (matrix(r,c) == 0) {
		if (r < meas_count && c < tracks_count) {
		  double cov = it_prev->getMeasToTrackMatchingCov();
		  double mahalanobis_dist = std::sqrt((std::pow((
		    meas.points[meas_it].x - it_prev->getPos().x), 2) +
		    std::pow((meas.points[meas_it].y - it_prev->getPos().y), 2)) / cov);
		  double dist = distanceBtwTwoPoints(meas.points[meas_it], it_prev->getPos());
		  
		  if ((mahalanobis_dist < mahalanobis_dist_gate &&
		  dist < 0.45 && it_prev->getObservations() == 0) ||
		  (mahalanobis_dist < mahalanobis_dist_gate &&
		  dist < 0.35 && it_prev->getObservations() > 0))
		  {
		      // Found an assignment. Update the new measurement
		      // with the track ID and age of older track. Add
		      // to fused list
		      it_prev->update(meas.points[meas_it]);
		      fused.push_back(*it_prev);
		  } else {
		      // TOO MUCH OF A JUMP IN POSITION
		      // Probably a missed track or a new track
		      it_prev->missed();
		      fused.push_back(*it_prev);
		      
		      // And a new track
		      fused.push_back(initLeg(meas.points[meas_it]));
		  }
		  
		} else if (r >= meas_count) {
		      it_prev->missed();
		      fused.push_back(*it_prev);
		} else if (c >= tracks_count) {
		      // Possible new track
		  fused.push_back(initLeg(meas.points[meas_it]));
		}
		break; // There is only one assignment per row
	  }
	  if (c < tracks_count-1) {
	      it_prev++;
	  }
	}
	if (r < meas_count-1) {
	      meas_it++;
	}
      }
  }

  
  unsigned int LegDetector::getNextLegId()
  {
    return next_leg_id++;
  }


  void LegDetector::pub_border_square()
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    if (isOnePersonToTrack) {
      marker.pose.position.x = (x_upper_limit_dynamic + x_lower_limit_dynamic) / 2;
      marker.pose.position.y = (y_upper_limit_dynamic + y_lower_limit_dynamic) / 2;
    } else {
      marker.pose.position.x = (x_upper_limit + x_lower_limit) / 2;
      marker.pose.position.y = (y_upper_limit + y_lower_limit) / 2;
    }
    marker.pose.position.z = 0.0;
    if (isOnePersonToTrack) {
      marker.scale.x = x_upper_limit_dynamic - x_lower_limit_dynamic;
      marker.scale.y = y_upper_limit_dynamic - y_lower_limit_dynamic;
    } else {
      marker.scale.x = x_upper_limit - x_lower_limit;
      marker.scale.y = y_upper_limit - y_lower_limit;
    }
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    tracking_area_pub.publish( marker );
  }


  void LegDetector::removeOldBoundingBox()
  {
      visualization_msgs::Marker marker;
      marker.header.frame_id = transform_link;
      marker.header.stamp = ros::Time();
      marker.ns = nh_.getNamespace();
      marker.id = 0;
      marker.action = visualization_msgs::Marker::DELETE;
//       bounding_box_pub.publish(marker);
  }
  
  unsigned int LegDetector::getNextTrackingZoneMarkerId()
  {
    return tracking_zone_next_marker_id++;
  }
  
  void LegDetector::vis_tracking_zones()
  {
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < tracking_zones.size(); i++) 
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = transform_link;
      marker.header.stamp = ros::Time();
      marker.ns = nh_.getNamespace();
      marker.id = getNextTrackingZoneMarkerId();
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = (tracking_zones[i].getXUpperLimit() + tracking_zones[i].getXLowerLimit()) / 2;
      marker.pose.position.y = (tracking_zones[i].getYUpperLimit() + tracking_zones[i].getYLowerLimit()) / 2;
      marker.pose.position.z = 0.0;
      marker.scale.x = tracking_zones[i].getXUpperLimit() - tracking_zones[i].getXLowerLimit();
      marker.scale.y = tracking_zones[i].getYUpperLimit() - tracking_zones[i].getYLowerLimit();
      marker.scale.z = 0.01;
      marker.color.a = 1.0;
      marker.color.g = 1.0;
      
      ma.markers.push_back(marker);
    }
    tracking_zone_pub.publish(ma);
  }
  
  void LegDetector::deleteOldMarkers()
  {
    removeOldBoundingBox();
    
    if (legs_marker_next_id != 0) {
      removeOldMarkers(0, legs_marker_next_id, legs_and_vel_direction_publisher);
      legs_marker_next_id = 0;
    }
    
    if (people_marker_next_id != 0) {
      removeOldMarkers(0, people_marker_next_id, people_pub);
      people_marker_next_id = 0;
    }
    
    if (tracking_zone_next_marker_id != 0) {
      removeOldMarkers(0, tracking_zone_next_marker_id, tracking_zone_pub);
      tracking_zone_next_marker_id = 0;
    }
  }
  
  void LegDetector::removeLastSeenPeoplePosition(int i)
  {
    if (i != lastSeenPeoplePositions.size() - 1)
    {
      std::tuple<unsigned int, unsigned int, Point> t = lastSeenPeoplePositions[lastSeenPeoplePositions.size() - 1];
      lastSeenPeoplePositions[i] = t;
    }
    lastSeenPeoplePositions.pop_back();
  }
  
  bool LegDetector::isPointNearToLimits(Point p)
  {
    return (std::abs(p.x - x_upper_limit) > 0.1 || 
	    std::abs(p.y - y_upper_limit) > 0.1 || 
	    std::abs(p.x - x_lower_limit) > 0.1 || 
	    std::abs(p.y - y_lower_limit) > 0.1);
  }
  
  void LegDetector::updateLastSeenPeoplePositions()
  {
    for (int i = 0; i < lastSeenPeoplePositions.size(); i++)
    {
      if (std::get<0>(lastSeenPeoplePositions[i]) * frequency > 5.0
	|| (std::get<0>(lastSeenPeoplePositions[i]) * frequency > 1.0 
	    && isPointNearToLimits(std::get<2>(lastSeenPeoplePositions[i])))) 
      { 
	int pId = std::get<1>(lastSeenPeoplePositions[i]);
	std::map<int, visualization_msgs::Marker>::iterator it = paths.find(pId);
	if (it != paths.end())
	{
	  paths.erase(it);
	}
	removeLastSeenPeoplePosition(i);
      } 
      else
      {
	std::get<0>(lastSeenPeoplePositions[i])++;
      }
    }
  }
  
  void LegDetector::updatePaths()
  {
    visualization_msgs::MarkerArray ma;
    std::map<int, visualization_msgs::Marker>::iterator it = paths.begin();
    for (; it != paths.end(); it++)
    {
      if (it->second.points.size() >= 80) 
      {
	std::rotate(it->second.points.begin(), it->second.points.begin() + 1, it->second.points.end());
	it->second.points.pop_back();
      }
	ma.markers.push_back(it->second);
    }
  }
  
  void LegDetector::pub_triangle()
  {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = transform_link;
    line_strip.header.stamp = ros::Time();
    line_strip.ns = nh_.getNamespace();
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.scale.x = 0.03;
    line_strip.color.a = 1.0;
    line_strip.color.r = getRandomNumberFrom0To1();
    line_strip.color.g = getRandomNumberFrom0To1();
    line_strip.color.b = getRandomNumberFrom0To1();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    geometry_msgs::Point p;
    p.x = 0.;
    p.y = 0.;
    line_strip.points.push_back(p);
    
    bool is_fst_leg = distanceBtwTwoPoints(p.x, p.y, legs[0].getPos().x, legs[0].getPos().y) <= distanceBtwTwoPoints(p.x, p.y, legs[1].getPos().x, legs[1].getPos().y);
    if (is_fst_leg) { p.x = legs[0].getPos().x; p.y = legs[0].getPos().y; }
    else { p.x = legs[1].getPos().x; p.y = legs[1].getPos().y; }
    p.x -= distanceBtwTwoPoints(legs[0].getPos(), legs[1].getPos());
    
    double to_add = 0.;
    if (is_fst_leg)
    {
       to_add = 0.07 * (1 + distanceBtwTwoPoints(legs[0].getPos(), legs[1].getPos()) / calculateNorm(legs[0].getPos()));
    }
    else
    {
      to_add = 0.07 * (1 + distanceBtwTwoPoints(legs[0].getPos(), legs[1].getPos()) / calculateNorm(legs[1].getPos()));
    }
    
    p.y -= to_add;
    
    line_strip.points.push_back(p);
    if (is_fst_leg) { p.x = legs[1].getPos().x; p.y = legs[1].getPos().y; }
    else { p.x = legs[0].getPos().x; p.y = legs[0].getPos().y; }
    p.x -= distanceBtwTwoPoints(legs[0].getPos(), legs[1].getPos());
    p.y += to_add;
    line_strip.points.push_back(p);
    p.x = 0.; p.y = 0.;
    line_strip.points.push_back(p);
    marker_pub.publish(line_strip);
  }

  void LegDetector::resetLeftRight()
  {
    left_right = std::make_pair(-1, -1);
    ij_or_ji = std::make_pair(0, 0);
    conf_left_right = 0.01;
  }
  
  leg_tracker::LegMsg LegDetector::getLegMsg(int leg_index)
  {
    leg_tracker::LegMsg legMsg;
    legMsg.ID = legs[leg_index].getLegId();
    legMsg.confidence = legs[leg_index].getConfidence();
    legMsg.position.x = legs[leg_index].getPos().x;
    legMsg.position.y = legs[leg_index].getPos().y;
    legMsg.velocity.x = legs[leg_index].getVel().x;
    legMsg.velocity.y = legs[leg_index].getVel().y;
    legMsg.acceleration.x = legs[leg_index].getAcc().x;
    legMsg.acceleration.y = legs[leg_index].getAcc().y;
    return legMsg;
  }
  
  void LegDetector::publish_person_msg_stamped(int fst_leg_index, int snd_leg_index, std_msgs::Header header) {
    leg_tracker::PersonMsg msg;
    msg.header = header;
    msg.ID = legs[fst_leg_index].getPeopleId();
    msg.leg1 = getLegMsg(fst_leg_index);
    msg.leg2 = getLegMsg(snd_leg_index);
    people_msg_pub.publish(msg);
  }

  void LegDetector::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    updateLastSeenPeoplePositions();
    
    if (isOnePersonToTrack && waitForTrackingZoneReset * frequency > 5.0) 
    {
      resetLeftRight();
      resetTrackingZone();
      waitForTrackingZoneReset = 0;
    }
    if (isOnePersonToTrack) { waitForTrackingZoneReset++; }
    
    for (int i = 0; i < tracking_zones.size(); i++)
    {
      tracking_zones[i].incrementWithoutUpdate();
      if (tracking_zones[i].isWithoutUpdate())
      {
	tracking_zones.erase(tracking_zones.begin() + i);
      }
    }
    
//     ros::Time lasttime=ros::Time::now();
    
    if (with_map) {
      if (!got_map) { return; }
    }
    
    pub_border_square();
    
    deleteOldMarkers();
    
    sensor_msgs::PointCloud2 cloudFromScan, tfTransformedCloud;

    if (!laserScanToPointCloud2(scan, cloudFromScan)) { predictLegs(); return; }

    if (!tfTransformOfPointCloud2(scan, cloudFromScan, tfTransformedCloud)) { predictLegs(); return; }

    pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(tfTransformedCloud, *pcl_pc2);

    PointCloud cloudXYZ, filteredCloudXYZ;
    pcl::fromPCLPointCloud2(*pcl_pc2, cloudXYZ);
    filteredCloudXYZ.header = cloudXYZ.header;
    if (!filterPCLPointCloud(cloudXYZ, filteredCloudXYZ)) { predictLegs(); return; }

    PointCloud cluster_centroids;
    
    if (!clustering(filteredCloudXYZ, cluster_centroids)) { predictLegs(); return; }
    if (cluster_centroids.points.size() == 0) { predictLegs(); return; }

    if (isOnePersonToTrack) 
    {
      matchClusterCentroidsToLegs(cluster_centroids);
    } 
    else if (isBoundingBoxTracking) 
    {
      boundingBoxTracking(cluster_centroids);
    } 
    else 
    {
      gnn_munkres(cluster_centroids);
    }
    visLegs();
    findPeople();
    if (isOnePersonToTrack && legs.size() == 2)
    {
      std::vector<double> current_state;
      double confidence;
      for (int i = 0; i < legs.size(); i++)
      {
        if (legs[i].getCurrentState(current_state))
        {
          current_state.push_back(legs[i].getLegId());
          current_state.push_back(legs[i].getPeopleId());
          confidence = legs[i].getConfidence();
          current_state.push_back(confidence);
          pub_leg_posvelacc(current_state, i, scan->header);
        }
      }
    }
    vis_people(scan->header);
//     if (isOnePersonToTrack && legs.size() == 2)
//     {
//       double n1 = calculateNorm(legs[0].getPos());
//       double n2 = calculateNorm(legs[1].getPos());
//     }
    vis_tracking_zones();
//     ros::Time currtime=ros::Time::now();
//     ros::Duration diff=currtime-lasttime;
    if (isOnePersonToTrack) { waitForTrackingZoneReset = 0; }
  }
