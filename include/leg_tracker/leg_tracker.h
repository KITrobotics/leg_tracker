
#ifndef LEG_TRACKER_H
#define LEG_TRACKER_H

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
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
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

#include <leg_tracker/munkres.h>
#include <leg_tracker/leg.h>
#include <leg_tracker/bounding_box.h>
#include <leg_tracker/LegTrackerMessage.h>
#include <leg_tracker/LegMsg.h>
#include <leg_tracker/PersonMsg.h>

typedef pcl::PointCloud<Point> PointCloud;


class LegDetector
{
private:
  ros::Subscriber sub;
  ros::Subscriber global_map_sub;
  laser_geometry::LaserProjection projector_;
  ros::Publisher pos_vel_acc_fst_leg_pub;
  ros::Publisher pos_vel_acc_snd_leg_pub;
  ros::Publisher legs_and_vel_direction_publisher;
  ros::Publisher tracking_area_pub;
  ros::Publisher people_pub;
  ros::Publisher fst_leg_msg_pub;
  ros::Publisher snd_leg_msg_pub;
  ros::Publisher people_msg_pub;
  
  ros::Publisher marker_pub;
  ros::Publisher cov_marker_pub;
//   ros::Publisher bounding_box_pub;
  ros::Publisher tracking_zone_pub;
//   ros::Publisher paths_publisher;
  
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

  unsigned int legs_marker_next_id;
  unsigned int people_marker_next_id;
  unsigned int next_leg_id;
  unsigned int cov_ellipse_id;
  unsigned int tracking_zone_next_marker_id;

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
  
  double ref_point_x;
  double ref_point_y;
  
  double waitForTrackingZoneReset;
  
  double frequency;
  
  nav_msgs::OccupancyGrid global_map;
  bool got_map;
  bool with_map;
  
  std::vector<Leg> legs;
  std::vector<Leg> removed_legs;
  
  std::vector<BoundingBox> tracking_zones;
  
  bool got_map_from_service;
  
  // counter, peopleId, last position
  std::vector<std::tuple<unsigned int, unsigned int, Point> > lastSeenPeoplePositions;
  
  std::map<int, visualization_msgs::Marker> paths;
  
  int checked, how_much_times_to_check;
  
  std::pair<int, int> left_right;
  double conf_left_right;
  std::pair<int, int> ij_or_ji;
  

public:
  ros::NodeHandle nh_;
  
  LegDetector(ros::NodeHandle nh);
  
  ~LegDetector() {}
  
  void init();
  
  double getRandomNumberFrom0To1();
  
  void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  
  double calculateNorm(Point p);
  
  visualization_msgs::Marker getCovarianceEllipse(int id, const double meanX, const double meanY, const Eigen::MatrixXd& S);
  
  bool laserScanToPointCloud2(const sensor_msgs::LaserScan::ConstPtr& scan, sensor_msgs::PointCloud2& cloud);
  
  bool tfTransformOfPointCloud2(const sensor_msgs::LaserScan::ConstPtr& scan, 
                                sensor_msgs::PointCloud2& from, sensor_msgs::PointCloud2& to);

  void pub_leg_posvelacc(std::vector<double>& in, bool isSnd, std_msgs::Header header);

  bool filterPCLPointCloud(const PointCloud& in, PointCloud& out);
  
  Leg initLeg(const Point& p);
  
  void printLegsInfo(std::vector<Leg> vec, std::string name);
  
  void printPointCloudPoints(PointCloud& cloud, std::string name);

  void removeOldMarkers(int from, int toExclusive, ros::Publisher ma_publisher);

  void visLegs();

  unsigned int getNextLegsMarkerId();

  visualization_msgs::Marker getArrowMarker(double start_x, double start_y, 
                                            double end_x, double end_y, int id);
  
  // only for the user of the robot platform
  void matchClusterCentroidsToLegs(PointCloud cluster_centroids, std::map<int, pcl::PointCloud<Point>>&  cluster_map);
    
  void resetTrackingZone();
  
  double how_much_in_free_space(double x, double y);
  
  void separateLegs(int i, int j);
  
  void checkDistanceOfLegs();

  void findPeople();

  void findSecondLeg(int fst_leg);
  
  double distanceBtwTwoPoints(double x1, double y1, double x2, double y2);
  
  double distanceBtwTwoPoints(Point p1, Point p2);

  void setPeopleId(int fst_leg, int snd_leg);
  
  void eraseRemovedLegsWithoutId();
  
  void eraseRemovedLeg(int id);
  
  visualization_msgs::Marker getOvalMarker(int id, double x, double y, 
      double orientation_x, double orientation_y, double orientation_z, double orientation_w,
      double scale_x, double scale_y, double r, double g, double b);

  visualization_msgs::Marker getOvalMarkerForTwoPoints(int pId, double x1, double y1, 
                                                       double x2, double y2, int id);

  void checkIfLeftOrRight(int i, int j);

  void vis_people(std_msgs::Header header);

  void pubExtendedLine(double x1, double y1, double x2, double y2, int id);
  
  void updatePath(unsigned int pId, std_msgs::Header header, 
                  double x1, double y1, double x2, double y2);

  unsigned int getPeopleMarkerNextId();

  bool clustering(const PointCloud& cloud, PointCloud& cluster_centroids, std::map<int, pcl::PointCloud<Point>>&  cluster_map);
  
  void pub_bounding_box(double min_x, double min_y, double max_x, double max_y);
  
  void pub_border(double min_x, double min_y, double max_x, double max_y);

  void predictLegs();

  void removeLegFromVector(std::vector<Leg>& v, unsigned int i);
  
  void resetHasPair(std::vector<Leg>& v, int fst_leg);

  void cullDeadTracks(std::vector<Leg>& v);

  unsigned int getNextCovEllipseId();
  
  void boundingBoxTracking(PointCloud& cluster_centroids);
  
  void matchClusterCentroids2Legs(unsigned int fst_leg_id, unsigned int snd_leg_id, 
                                  PointCloud& cluster_centroids, int tracking_zone_index);

  void gnn_munkres(PointCloud& cluster_centroids);
    
  void assign_munkres(const PointCloud& meas, std::vector<Leg> &tracks, std::vector<Leg> &fused);
    
  unsigned int getNextLegId();
  
  void pub_border_square();
  
  void removeOldBoundingBox();
    
  unsigned int getNextTrackingZoneMarkerId();

  void vis_tracking_zones();
    
  void deleteOldMarkers();
    
  void removeLastSeenPeoplePosition(int i);
  
  bool isPointNearToLimits(Point p);
  
  void updateLastSeenPeoplePositions();
  
  void updatePaths();
  
  void pub_triangle();
  
  void resetLeftRight();

  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
  
  void publish_person_msg_stamped(int fst_leg_index, int snd_leg_index, std_msgs::Header header);
  
  leg_tracker::LegMsg getLegMsg(int leg_index);
};

#endif
