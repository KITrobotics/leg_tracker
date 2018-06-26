
#include <algorithm>
#include <vector>
#include <iostream>
#include <math.h>
#include <cstdlib>
#include <fstream>
#include <tuple>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/GetMap.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iirob_filters/kalman_filter.h>
#include <iirob_filters/KalmanFilterParameters.h>

#include "munkres.h"
#include "leg.h"
#include "bounding_box.h"



typedef pcl::PointCloud<Point> PointCloud;


class LegDetector
{
private:
  ros::Subscriber sub;
  ros::Subscriber global_map_sub;
  laser_geometry::LaserProjection projector_;
  ros::Publisher sensor_msgs_point_cloud_publisher;
  ros::Publisher pcl_cloud_publisher;
  ros::Publisher pos_vel_acc_lleg_pub;
  ros::Publisher pos_vel_acc_rleg_pub;
  ros::Publisher legs_and_vel_direction_publisher;
  ros::Publisher tracking_area_pub;
  ros::Publisher people_pub;
  
  ros::Publisher marker_pub;
  ros::Publisher cov_marker_pub;
  ros::Publisher bounding_box_pub;
  ros::Publisher tracking_zone_pub;
  
  ros::ServiceClient client; 
  nav_msgs::GetMap srv;


  std::string transform_link;
  std::string scan_topic;
  std::string global_map_topic;
  
  double x_lower_limit;
  double x_upper_limit;
  double y_lower_limit;
  double y_upper_limit;
  
  double x_lower_limit_dynamic;
  double x_upper_limit_dynamic;
  double y_lower_limit_dynamic;
  double y_upper_limit_dynamic;
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  double ransac_dist_threshold;
  std::string circle_fitting;
  double leg_radius;
  std::vector<double> centerOfLegLastMeasurement;
  Point person_center;
  int legs_gathered;
  int min_observations;
  double max_dist_btw_legs;
  int id_counter;
  double z_coordinate;
  double vel_stance_threshold;
  double vel_swing_threshold;
  int state_dimensions;
  int minClusterSize;
  int maxClusterSize;
  double clusterTolerance;
  int occluded_dead_age;
  double variance_observation;

  bool isOnePersonToTrack;
  bool isBoundingBoxTracking;

  int legs_marker_next_id;
  int people_marker_next_id;
  int next_leg_id;
  int cov_ellipse_id;
  int tracking_zone_next_marker_id;

  double max_cov;
  double min_dist_travelled;
  double in_free_space_threshold;
  
    
  double mahalanobis_dist_gate;
  double euclidian_dist_gate;
  double max_cost;
  double tracking_bounding_box_uncertainty;
  double cluster_bounding_box_uncertainty;
  double outlier_removal_radius;
  int max_neighbors_for_outlier_removal;
  
  double ellipse_x;
  double ellipse_y;
  double ellipse_rx;
  double ellipse_ry;
  
  double waitForTrackingZoneReset;
  
  double frequency;
  
  nav_msgs::OccupancyGrid global_map;
  bool got_map;
  bool with_map;
  
  std::vector<Leg> legs;
  std::vector<Leg> removed_legs;
  
  std::vector<BoundingBox> tracking_zones;
  
  bool got_map_from_service;
  
  std::vector<std::tuple<unsigned int, unsigned int, Point> > lastSeenPeoplePositions;
  

public:
  ros::NodeHandle nh_;
  LegDetector(ros::NodeHandle nh) : nh_(nh), /*params_{std::string(nh_.getNamespace() + "/KalmanFilter")}, */
      tfListener(tfBuffer)
  {
    init();

  }
  ~LegDetector() {}

  void init()
  { 
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
    sensor_msgs_point_cloud_publisher = nh_.advertise<sensor_msgs::PointCloud2> ("scan2cloud", 300);
    pcl_cloud_publisher = nh_.advertise<PointCloud> ("scan2pclCloud", 300);
    pos_vel_acc_lleg_pub = nh_.advertise<std_msgs::Float64MultiArray>("pos_vel_acc_lleg", 300);
    pos_vel_acc_rleg_pub = nh_.advertise<std_msgs::Float64MultiArray>("pos_vel_acc_rleg", 300);
    legs_and_vel_direction_publisher = nh_.advertise<visualization_msgs::MarkerArray>("legs_and_vel_direction", 300);
    tracking_area_pub = nh_.advertise<visualization_msgs::Marker>("tracking_area", 300);
    people_pub = nh_.advertise<visualization_msgs::MarkerArray>("people", 300);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("line_strip", 10);
    cov_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("cov_ellipses", 10);
    
    bounding_box_pub = nh_.advertise<visualization_msgs::Marker>("bounding_box", 300);
    tracking_zone_pub = nh_.advertise<visualization_msgs::MarkerArray>("tracking_zones", 100);
    
    client = nh_.serviceClient<nav_msgs::GetMap>("static_map");
  }
  
  
  void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
  {
    global_map = *msg;
    if (!got_map) { got_map = true; }
  }
  

  double calculateNorm(Point p)
  {
    return std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));
  }
  

  visualization_msgs::Marker getCovarianceEllipse(int id, const double meanX, const double meanY, const Eigen::MatrixXd& S)
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


  bool laserScanToPointCloud2(const sensor_msgs::LaserScan::ConstPtr& scan, sensor_msgs::PointCloud2& cloud)
  {
    if (!scan) { ROS_DEBUG("Laser scan pointer was not set!"); return false; }
    projector_.projectLaser(*scan, cloud);
    return true;
  }

  
  bool tfTransformOfPointCloud2(const sensor_msgs::LaserScan::ConstPtr& scan,
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


  // pass through filtering, outlier removal
  bool filterPCLPointCloud(const PointCloud& in, PointCloud& out)
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
    
    if (with_map && got_map) {
      
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
    
    return true;
  }

  
  Leg initLeg(const Point& p)
  {
    Leg l(getNextLegId(), p, occluded_dead_age,
      variance_observation, min_observations, state_dimensions, min_dist_travelled);
    return l;
  }


  void printLegsInfo(std::vector<Leg> vec, std::string name)
  {
    ROS_INFO("%s:", name.c_str());
    for (Leg& l : vec)
    {
      ROS_INFO("legId: %d, peopleId: %d, pos: (%f, %f), observations: %d, hasPair: %d, missed: %d, dist_traveled: %f",
      l.getLegId(), l.getPeopleId(), l.getPos().x, l.getPos().y, l.getObservations(), l.hasPair(), l.getOccludedAge(), l.getDistanceTravelled());
    }
  }
  
  void printPoinCloudPoints(PointCloud& cloud, std::string name)
  {
    ROS_INFO("cloud %s: ", name.c_str());
    int i = 0;
    for (Point& p : cloud.points)
    {
      ROS_INFO("point: %d = (%f, %f)", i, p.x, p.y);
      i++;
    }
  }
  

  void removeOldMarkers(int nextId, ros::Publisher ma_publisher)
  {
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < nextId; i++) {
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
  

  void visLegs()
  {
    visualization_msgs::MarkerArray ma_leg;
    for (Leg& l : legs)
    {
      double pos_x = l.getPos().x;
      double pos_y = l.getPos().y;
      ma_leg.markers.push_back(getArrowMarker(pos_x, pos_y,
	       pos_x + 0.5 * l.getVel().x, pos_y + 0.5 * l.getVel().y, getNextLegsMarkerId()));
    }
    legs_and_vel_direction_publisher.publish(ma_leg);
  }

  unsigned int getNextLegsMarkerId()
  {
    return legs_marker_next_id++;
  }


  visualization_msgs::Marker getArrowMarker(double start_x, double start_y, double end_x, double end_y, int id)
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
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    return marker;
  }

  
  visualization_msgs::Marker getMarker(double x, double y, double scale_z, int id)
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
    marker.pose.position.z = z_coordinate / 2;
    marker.scale.x = leg_radius;
    marker.scale.y = leg_radius;
    marker.scale.z = scale_z;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    return marker;
  }
  

  // only for the user of the robot platform
  void matchClusterCentroidsToLegs(PointCloud cluster_centroids)
  {
    bool toReset = false;
    for (int i = 0; i < legs.size(); i++)
    {
      if (!legs[i].is_dead()) {
	legs[i].predict();
	if (legs[i].getPos().x > x_upper_limit || legs[i].getPos().y > y_upper_limit || 
	  legs[i].getPos().x < x_lower_limit || legs[i].getPos().y < y_lower_limit)
	{
	  toReset = true;
	}
      }
    }
    
    if (toReset) 
    {
      legs.clear();
      resetTrackingZone();
      return;
    }
    
    
    if (cluster_centroids.points.size() == 0) { return; }
    
    if (cluster_centroids.points.size() == 1) {
      Point p = cluster_centroids.points[0];
      double min_dist = max_cost;
      int index = -1;
      for (int i = 0; i < legs.size(); i++) {
	double cov = legs[i].getMeasToTrackMatchingCov();
	double mahalanobis_dist = std::sqrt((std::pow((p.x - legs[i].getPos().x), 2) +
	  std::pow((p.y - legs[i].getPos().y), 2)) / cov);
	double euclid_dist = distanceBtwTwoPoints(p, legs[i].getPos());
	if (mahalanobis_dist < min_dist && euclid_dist < 0.25)
	{
	  index = i;
	  min_dist = mahalanobis_dist;
	}
      }
      if (index != -1)  
      { 
	legs[index].update(p);
      }	
    } else if (legs.size() == 1) {
      double min_dist = max_cost;
      int index = -1;
      for (int i = 0; i < cluster_centroids.points.size(); i++) {
	double cov = legs[i].getMeasToTrackMatchingCov();
	double mahalanobis_dist = std::sqrt((std::pow((cluster_centroids.points[i].x - legs[i].getPos().x), 2) +
	  std::pow((cluster_centroids.points[i].y - legs[i].getPos().y), 2)) / cov);
	double euclid_dist = distanceBtwTwoPoints(cluster_centroids.points[i], legs[i].getPos());
	if (mahalanobis_dist < min_dist && euclid_dist < 0.25)
	{
	  index = i;
	  min_dist = mahalanobis_dist;
	}
      }
      if (index != -1)  
      { 
	legs[0].update(cluster_centroids.points[index]);
      }	
    } else if (legs.size() == 2 && cluster_centroids.points.size() > 1) {
      
      double total_cost = max_cost;
      double fst_cov = legs[0].getMeasToTrackMatchingCov();
      double snd_cov = legs[1].getMeasToTrackMatchingCov();
      int fst_index = -1, snd_index = -1;
      
      for (int i = 0; i < cluster_centroids.points.size(); i++) {
	double euclid_dist = distanceBtwTwoPoints(cluster_centroids.points[i], legs[0].getPos());
// 	if (euclid_dist > 0.25 && cluster_centroids.points.size() ) { continue; }
	double fst_cost = std::sqrt((std::pow((cluster_centroids.points[i].x - legs[0].getPos().x), 2) +
	  std::pow((cluster_centroids.points[i].y - legs[0].getPos().y), 2)) / fst_cov);
	for (int j = 0; j < cluster_centroids.points.size(); j++) {
	  if (i == j) { continue; }
	  double snd_cost = std::sqrt((std::pow((cluster_centroids.points[j].x - legs[i].getPos().x), 2) +
	    std::pow((cluster_centroids.points[j].y - legs[i].getPos().y), 2)) / snd_cov);
	  if (fst_cost + snd_cost < total_cost) {
	    total_cost = fst_cost + snd_cost;
	    fst_index = i;
	    snd_index = j;
	  }
	}
      }
      
      if (fst_index == -1 || snd_index == -1) { return; }
      
      legs[0].update(cluster_centroids.points[fst_index]);
      legs[1].update(cluster_centroids.points[snd_index]);
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
      
//       ROS_INFO("x_lower_limit_dynamic: %f, x_upper_limit_dynamic: %f, y_lower_limit_dynamic: %f, y_upper_limit_dynamic: %f, flaeche: %f", 
// 	       x_lower_limit_dynamic, x_upper_limit_dynamic, y_lower_limit_dynamic, y_upper_limit_dynamic, 
// 	       (x_upper_limit_dynamic - x_lower_limit_dynamic) * (y_upper_limit_dynamic - y_lower_limit_dynamic));
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


    if (legs.size() < 2) {
      for (Point& p : cluster_centroids.points)
      {
	legs.push_back(initLeg(p));
	if (legs.size() >= 2) { break; }
      }
    }
  }
  
  
  void resetTrackingZone() 
  {
    x_lower_limit_dynamic = x_lower_limit;
    x_upper_limit_dynamic = x_upper_limit;
    y_lower_limit_dynamic = y_lower_limit;
    y_upper_limit_dynamic = y_upper_limit;
  }
  
  
  double how_much_in_free_space(double x, double y)
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


  void separateLegs(int i, int j)
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

  
  void checkDistanceOfLegs()
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
  

  void findPeople()
  {
    checkDistanceOfLegs();
    for (int i = 0; i < legs.size(); i++)
    {
      if (legs[i].hasPair() || (legs[i].getPeopleId() == -1 && legs[i].getObservations() < min_observations)) 
	{ continue; }
      findSecondLeg(i);
    }
  }

  void findSecondLeg(int fst_leg)
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
  

  double distanceBtwTwoPoints(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
  }

  
  double distanceBtwTwoPoints(Point p1, Point p2)
  {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
  }


  void setPeopleId(int fst_leg, int snd_leg)
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
      
      ROS_WARN("new peoplePos: (%f, %f), lastPos: (%f, %f), last id: %d", peoplePos.x, peoplePos.y, lastPos.x, lastPos.y, id);
      
      marker_pub.publish(getArrowMarker(3 * peoplePos.x, 3 * peoplePos.y, 3 * lastPos.x, 3 * lastPos.y, 999999));
      
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

  void eraseRemovedLegsWithoutId()
  {
    for (std::vector<Leg>::iterator it = removed_legs.begin(); it != removed_legs.end(); it++)
    {
      if (it->getPeopleId() == -1)
      {
	removed_legs.erase(it);
      }
    }
  }

  void eraseRemovedLeg(int id)
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
  
  
  visualization_msgs::Marker getOvalMarker(int id, double x, double y, 
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
//     marker.scale.z = 0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
  }
  

  visualization_msgs::Marker getOvalMarkerForTwoPoints(double x1, double y1, double x2, double y2, int id)
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
    
    return getOvalMarker(id, x, y, orientation_x, orientation_y, orientation_z, orientation_w,
      scale_x, scale_y, 0.0, 0.0, 0.0);
  }


  void vis_people()
  {
    visualization_msgs::MarkerArray ma_people;

    for (int i = 0; i < legs.size(); i++)
    {
      int id = legs[i].getPeopleId();
      if (id == -1) { continue; }

      	// second leg is removed
	if (!legs[i].hasPair())
	{
	  for (Leg& l : removed_legs)
	  {
	    if (l.getPeopleId() == id)
	    {
	      if (distanceBtwTwoPoints(legs[i].getPos(), l.getPos()) > max_dist_btw_legs) {
		break;
	      }
	      ma_people.markers.push_back(getOvalMarkerForTwoPoints(
		legs[i].getPos().x,
		legs[i].getPos().y,
		l.getPos().x, 
		l.getPos().y, 
		getPeopleMarkerNextId()));

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
	      ma_people.markers.push_back(getOvalMarkerForTwoPoints(legs[i].getPos().x,
		  legs[i].getPos().y, legs[j].getPos().x, legs[j].getPos().y, getPeopleMarkerNextId()));
	      break;
	    }
	  }
      }
    }

    people_pub.publish(ma_people);
  }


  unsigned int getPeopleMarkerNextId() {
    return people_marker_next_id++;
  }


  bool clustering(const PointCloud& cloud, PointCloud& cluster_centroids)
  {
//     pcl_cloud_publisher.publish(cloud);
//     pubWithZCoordinate(cloud);

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
      
      
      /*
      if (with_map && got_map) {
	
	bool isPointTransformed = true;
	geometry_msgs::PointStamped point_in, point_out;
	
	point_in.header.frame_id = cloud.header.frame_id;
	point_in.header.stamp = ros::Time::now();
	point_in.point.x = p.x; 
	point_in.point.y = p.y;
	
	geometry_msgs::TransformStamped transformStamped;
	try{
	  transformStamped = tfBuffer.lookupTransform(global_map.header.frame_id, point_in.header.frame_id, ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
	  ROS_WARN("Failure to lookup the transform for a point! %s\n", ex.what());
	  isPointTransformed = false;
	}
	tf2::doTransform(point_in, point_out, transformStamped);
    
	if (isPointTransformed) {
	  double in_free_space = how_much_in_free_space(point_out.point.x, point_out.point.y);
	  if (in_free_space > in_free_space_threshold) 
	  {
	    continue;
	  } 
	}
      }*/
      
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
    
    
    
    /* 
     * if there are two legs for one cluster then the cluster must be divided
     */
    
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
	pub_bounding_box(minMaxPoints[i].first.x, minMaxPoints[i].first.y, 
	  minMaxPoints[i].second.x, minMaxPoints[i].second.y);
	
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
//     pubWithZCoordinate(cluster_centroids);
//     pcl_cloud_publisher.publish(cluster_centroids);
    return true;
  }
  
  
  
  void pub_bounding_box(double min_x, double min_y, double max_x, double max_y)
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
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.g = 1.0;
    bounding_box_pub.publish( marker );
  }
  
  void pub_border(double min_x, double min_y, double max_x, double max_y)
  {	
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
//     marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (max_x + min_x) / 2;
    marker.pose.position.y = (max_y + min_y) / 2;
    marker.pose.position.z = z_coordinate;
    marker.scale.x = max_x - min_x;
    marker.scale.y = max_y - min_y;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.g = 1.0;
    tracking_area_pub.publish( marker );
  }
  

  void predictLegs()
  {
    for (int i = 0; i < legs.size(); i++) {
      legs[i].predict(); 
      legs[i].missed();
    }
    cullDeadTracks(legs);
  }

  void removeLegFromVector(std::vector<Leg>& v, unsigned int i)
  {
    // save people id at v[i] 
    if (v[i].getPeopleId() != -1 && !v[i].hasPair()) 
    {
      
	printLegsInfo(removed_legs, "removed_legs to find id " + std::to_string(v[i].getPeopleId()));
      // find second leg and save people position with people id
      for (int j = 0; j < removed_legs.size(); j++) 
      {
	if (removed_legs[j].getPeopleId() != v[i].getPeopleId()) { continue; }
	
	Point peoplePos;
	peoplePos.x = (v[i].getPos().x + removed_legs[j].getPos().x) / 2;
	peoplePos.y = (v[i].getPos().y + removed_legs[j].getPos().y) / 2;
	
	ROS_WARN("hier peoplePos: %f, %f", peoplePos.x, peoplePos.y);
	
	lastSeenPeoplePositions.push_back(std::make_tuple(0, v[i].getPeopleId(), peoplePos));
      }
    }
      
    if (i != v.size() - 1) {
      Leg l = v[v.size() - 1];
      v[i] = l;
    }
    v.pop_back();
  }

  void resetHasPair(std::vector<Leg>& v, int fst_leg)
  {
    for (int snd_leg = 0; snd_leg < v.size(); snd_leg++) {
      if (snd_leg != fst_leg && v[snd_leg].getPeopleId() == v[fst_leg].getPeopleId()) {
	v[snd_leg].setHasPair(false);
	break;
      }
    }
  }

  void cullDeadTracks(std::vector<Leg>& v)
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

  int getNextCovEllipseId()
  {
    return cov_ellipse_id++;
  }
  
  void boundingBoxTracking(PointCloud& cluster_centroids)
  {
//     if (legs.size() == 0)
//     {
//       ROS_INFO("there are no legs at beginning");
//       for (int i = 0; i < cluster_centroids.size(); i++)
//       {
// 	legs.push_back(initLeg(cluster_centroids.points[i]));
//       }
//       return;
//     }
    
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
// 	if (legs[i].getPos().x > x_upper_limit || legs[i].getPos().y > y_upper_limit || 
// 	  legs[i].getPos().x < x_lower_limit || legs[i].getPos().y < y_lower_limit)
// 	{
// 	  toReset = true;
// 	}
    
    printPoinCloudPoints(cluster_centroids, "cluster_centroids");
    /*
     * reset tracking zone or remove track
     */
//     if (toReset) 
//     {
//       legs.clear();
//       resetTrackingZone();
//       return;
//     }
    
    if (cluster_centroids.points.size() == 0) { return; }
    
    std::map<int, bool> usedPointsIds;
    
    for (int i = 0; i < tracking_zones.size(); i++)
    {
      ROS_INFO("tracking_zone %d, peopleId: %d, fst_leg_id: %d, snd_leg_id: %d", i, tracking_zones[i].getPeopleId(),
	tracking_zones[i].getFstLegId(), tracking_zones[i].getSndLegId());
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
      printPoinCloudPoints(used_points_cloud, "used_points_cloud " + std::to_string(i));
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
    
    printPoinCloudPoints(rest_points, "rest_points");
    
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
    
    printLegsInfo(legsWithoudPeopleId, "legsWithoudPeopleId before munkres");

    assign_munkres(rest_points, legsWithoudPeopleId, fused);
    
    printLegsInfo(legsWithoudPeopleId, "legsWithoudPeopleId after munkres");

    cullDeadTracks(fused);
    
    for (int i = 0; i < fused.size(); i++) 
    {
      legsWithPeopleId.push_back(fused[i]);
    }
    
    legs = legsWithPeopleId;
  }
  
  void matchClusterCentroids2Legs(unsigned int fst_leg_id, unsigned int snd_leg_id, PointCloud& cluster_centroids, int tracking_zone_index)
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
// 	double fst_euclid_dist = distanceBtwTwoPoints(cluster_centroids.points[i], legs[fst_leg].getPos());
// 	if (fst_euclid_dist > 0.3) 
// 	{  
// 	  fst_cost = max_cost;
// 	}
	for (int j = 0; j < cluster_centroids.points.size(); j++) 
	{
	  if (i == j) { continue; }
	  double snd_cost = std::sqrt((std::pow((cluster_centroids.points[j].x - legs[i].getPos().x), 2) +
	    std::pow((cluster_centroids.points[j].y - legs[i].getPos().y), 2)) / snd_cov);
// 	  double snd_euclid_dist = distanceBtwTwoPoints(cluster_centroids.points[j], legs[snd_leg].getPos());
// 	  if (snd_euclid_dist > 0.3) 
// 	  {  
// 	    snd_cost = max_cost;
// 	  }
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
	ROS_INFO("Match fst point (%f, %f) to leg with id %d and position (%f, %f)", cluster_centroids.points[fst_index].x, cluster_centroids.points[fst_index].y,
		 legs[fst_leg].getLegId(), legs[fst_leg].getPos().x, legs[fst_leg].getPos().y);
	legs[fst_leg].update(cluster_centroids.points[fst_index]);
      }
      
      legs[snd_leg].update(cluster_centroids.points[snd_index]);
      euclid_dist = distanceBtwTwoPoints(cluster_centroids.points[snd_index], legs[snd_leg].getPos());
      if (euclid_dist < 0.33) 
      {
	ROS_INFO("Match snd point (%f, %f) to leg with id %d and position (%f, %f)", cluster_centroids.points[snd_index].x, cluster_centroids.points[snd_index].y,
		 legs[snd_leg].getLegId(), legs[snd_leg].getPos().x, legs[snd_leg].getPos().y);
	legs[snd_leg].update(cluster_centroids.points[snd_index]);
      }
    } 
    
    
    /*
     * udpate tracking zone
     */
    tracking_zones[tracking_zone_index].update(legs[fst_leg].getPos().x, legs[fst_leg].getPos().y, legs[snd_leg].getPos().x, legs[snd_leg].getPos().y);
    
    /*
    if (legs.size() == 2) {
      double uncertainty = 0.2;
      x_lower_limit_dynamic = std::min(legs[fst_leg].getPos().x, legs[snd_leg].getPos().x);
      x_lower_limit_dynamic -= uncertainty;
      
      x_upper_limit_dynamic = std::max(legs[fst_leg].getPos().x, legs[snd_leg].getPos().x);
      x_upper_limit_dynamic += uncertainty;
      
      y_lower_limit_dynamic = std::min(legs[fst_leg].getPos().y, legs[snd_leg].getPos().y);
      y_lower_limit_dynamic -= uncertainty;
      
      y_upper_limit_dynamic = std::max(legs[fst_leg].getPos().y, legs[snd_leg].getPos().y);
      y_upper_limit_dynamic += uncertainty;
      */
//       ROS_INFO("x_lower_limit_dynamic: %f, x_upper_limit_dynamic: %f, y_lower_limit_dynamic: %f, y_upper_limit_dynamic: %f, flaeche: %f", 
// 	       x_lower_limit_dynamic, x_upper_limit_dynamic, y_lower_limit_dynamic, y_upper_limit_dynamic, 
// 	       (x_upper_limit_dynamic - x_lower_limit_dynamic) * (y_upper_limit_dynamic - y_lower_limit_dynamic));
     
  /*   
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
      }*/
//     }

    
    /*
     * do something for legs withoud people id
     */
//     if (legs.size() < 2) {
//       for (Point& p : cluster_centroids.points)
//       {
// 	legs.push_back(initLeg(p));
// 	if (legs.size() >= 2) { break; }
//       }
//     }
  }
  

  void gnn_munkres(PointCloud& cluster_centroids)
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

  
  void assign_munkres(const PointCloud& meas,
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
	removeOldMarkers(cov_ellipse_id, cov_marker_pub);
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
	  cov_ellipse_ma.markers.push_back(getCovarianceEllipse(getNextCovEllipseId(), it_prev->getPos().x,
	    it_prev->getPos().y, cov_matrix));
	  
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
      
      cov_marker_pub.publish(cov_ellipse_ma);
      
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

  
  unsigned int getNextLegId()
  {
    return next_leg_id++;
  }


  void pub_border_square()
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
//     marker.id = id;
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
//     marker.scale.z = 0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.g = 1.0;
    tracking_area_pub.publish( marker );
  }


  void removeOldBoundingBox()
  {
      visualization_msgs::Marker marker;
      marker.header.frame_id = transform_link;
      marker.header.stamp = ros::Time();
      marker.ns = nh_.getNamespace();
      marker.id = 0;
      marker.action = visualization_msgs::Marker::DELETE;
      bounding_box_pub.publish(marker);
  }
  
  unsigned int getNextTrackingZoneMarkerId()
  {
    return tracking_zone_next_marker_id++;
  }
  
  void vis_tracking_zones()
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
  //     marker.scale.z = 0;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.g = 1.0;
      
      ma.markers.push_back(marker);
    }
    tracking_zone_pub.publish(ma);
  }
  
  void deleteOldMarkers()
  {
    removeOldBoundingBox();
    
    if (legs_marker_next_id != 0) {
      removeOldMarkers(legs_marker_next_id, legs_and_vel_direction_publisher);
      legs_marker_next_id = 0;
    }
    
    if (people_marker_next_id != 0) {
      removeOldMarkers(people_marker_next_id, people_pub);
      people_marker_next_id = 0;
    }
    
    if (tracking_zone_next_marker_id != 0) {
      removeOldMarkers(tracking_zone_next_marker_id, tracking_zone_pub);
      tracking_zone_next_marker_id = 0;
    }
  }
  
  void removeLastSeenPeoplePosition(int i)
  {
    if (i != lastSeenPeoplePositions.size() - 1)
    {
      std::tuple<unsigned int, unsigned int, Point> t = lastSeenPeoplePositions[lastSeenPeoplePositions.size() - 1];
      lastSeenPeoplePositions[i] = t;
    }
    lastSeenPeoplePositions.pop_back();
  }
  
  void updateLastSeenPeoplePositions()
  {
    for (int i = 0; i < lastSeenPeoplePositions.size(); i++)
    {
      if (std::get<0>(lastSeenPeoplePositions[i]) * frequency < 5.0) 
      { 
	std::get<0>(lastSeenPeoplePositions[i])++;
      } 
      else
      {
	removeLastSeenPeoplePosition(i);
      }
    }
  }

  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    updateLastSeenPeoplePositions();
    for (int i = 0; i < lastSeenPeoplePositions.size(); i++)
    {
      ROS_WARN("time: %d, diff: %f, id: %d, pos: (%f, %f)", std::get<0>(lastSeenPeoplePositions[i]), 
							    5.0 - std::get<0>(lastSeenPeoplePositions[i]) * frequency, 
	       std::get<1>(lastSeenPeoplePositions[i]), std::get<2>(lastSeenPeoplePositions[i]).x, std::get<2>(lastSeenPeoplePositions[i]).y);
    }
//     if (!got_map_from_service) { 
//       if (client.call(srv))
//       {
// 	ROS_INFO("Service GetMap succeeded.");
// 	got_map_from_service = true;
//       }
//       else
//       {
// 	ROS_ERROR("Service GetMap failed.");
//       }
//     }
//     ROS_WARN("New Scan");
//     for (int i = 0; i < tracking_zones.size(); i++) {
//       ROS_INFO("tracking_zone %d, peopleId: %d, fst_leg_id: %d, snd_leg_id: %d", i, tracking_zones[i].getPeopleId(),
// 	tracking_zones[i].getFstLegId(), tracking_zones[i].getSndLegId());
//     }
//     printLegsInfo(legs, "legs before");
    // if there are no leg clutsters longer than 5 seconds the tracking zone will be reset
    if (isOnePersonToTrack && waitForTrackingZoneReset * frequency > 5.0) 
    {
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
    
    ros::Time lasttime=ros::Time::now();
    
    if (with_map) {
      if (!got_map) { return; }
    }
    
    pub_border_square();
    
    deleteOldMarkers();
    
    sensor_msgs::PointCloud2 cloudFromScan, tfTransformedCloud;

    if (!laserScanToPointCloud2(scan, cloudFromScan)) { predictLegs(); return; }

    if (!tfTransformOfPointCloud2(scan, cloudFromScan, tfTransformedCloud)) { predictLegs(); return; }

    sensor_msgs_point_cloud_publisher.publish(tfTransformedCloud);

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
    vis_people();
    vis_tracking_zones();
    ros::Time currtime=ros::Time::now();
    ros::Duration diff=currtime-lasttime;
//     std::cout << "Time for processing current laser scan: " << diff << std::endl;
    if (isOnePersonToTrack) { waitForTrackingZoneReset = 0; }
    printLegsInfo(legs, "legs after");
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv,"leg_tracker");
  ros::NodeHandle nh("~");
  LegDetector ld(nh);
  ros::spin();
  return 0;
}
