#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <vector>
#include "std_msgs/String.h"
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include "opencv2/core/version.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <iostream>
#include <iirob_filters/kalman_filter.h>
#include <pcl/common/centroid.h>
#include <iirob_filters/KalmanFilterParameters.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <leg.h>
#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <person.h>
#include <cstdlib>
//#include "Matrix.h"
#include <munkres.h>


/*
 * TODO:
 * N1 ~ new people appeared
 * N2 ~ continue tracking of already tracked people which were occluded
 * N1 = 0.7 and N2 = 1.2
 *
 * Invarianten erstellen:
 * if scan is not valid or scan does not have enough points -> calculate predictions
 *
 *
 *
 * Invarianten erstellen:
 * if scan is not valid or scan does not have enough points
 *
 *
 */




//export typedef Point;
typedef pcl::PointCloud<Point> PointCloud;
//typedef std::map<int, std::pair<Point, Point> > PeopleMap;

const int cluster_size = 2;
double mahalanobis_dist_gate = 1.6448536269514722;
double max_cost = 9999999.;

class LegDetector
{
private:
  ros::Subscriber sub;
  laser_geometry::LaserProjection projector_;
  ros::Publisher sensor_msgs_point_cloud_publisher;
  ros::Publisher pcl_cloud_publisher;
  ros::Publisher pos_vel_acc_lleg_pub;
  ros::Publisher pos_vel_acc_rleg_pub;
  ros::Publisher legs_and_vel_direction_publisher;
  ros::Publisher tracking_area_pub;
  ros::Publisher people_pub;

  std::string transform_link;
  std::string scan_topic;
  double x_lower_limit;
  double x_upper_limit;
  double y_lower_limit;
  double y_upper_limit;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  ros::Publisher vis_pub;
  double ransac_dist_threshold;
  std::string circle_fitting;
  double leg_radius;
  std::vector<double> centerOfLegLastMeasurement;
  Point person_center;
//   cv::Point2f left_leg_prediction;
//   cv::Point2f right_leg_prediction;
  int legs_gathered;
  int min_observations;
  // int min_predictions;
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

  int legs_marker_next_id;
  int people_marker_next_id;
  int next_leg_id;

  double max_nn_gating_distance;

  double min_dist_travelled;

  //mht
//   std::list<Leg> legs;
  std::vector<Leg> legs;


  //PeopleMap persons;
  std::vector<Leg> removed_legs;
  std::vector<Person> persons;
//   std::vector<std::pair<Leg, Leg> > persons;

  std::vector<Leg> temporary_legs;

//   iirob_filters::KalmanFilterParameters params_;

  // Construct the filter
  iirob_filters::MultiChannelKalmanFilter<double>* left_leg_filter;
  iirob_filters::MultiChannelKalmanFilter<double>* right_leg_filter;

public:
  ros::NodeHandle nh_;
  LegDetector(ros::NodeHandle nh) : nh_(nh), /*params_{std::string(nh_.getNamespace() + "/KalmanFilter")}, */
      tfListener(tfBuffer)
  {
    init();
//     left_leg_filter = new iirob_filters::MultiChannelKalmanFilter<double>();
//     if (!left_leg_filter->configure()) { ROS_ERROR("Configure of filter has failed!"); nh_.shutdown(); }
//     right_leg_filter = new iirob_filters::MultiChannelKalmanFilter<double>();
//     if (!right_leg_filter->configure()) { ROS_ERROR("Configure of filter has failed!"); nh_.shutdown(); }

  }
  ~LegDetector() {}

  void init()
  {
    std::srand(1);

    nh_.param("scan_topic", scan_topic, std::string("/base_laser_rear/scan"));
    nh_.param("transform_link", transform_link, std::string("base_link"));
    nh_.param("x_lower_limit", x_lower_limit, 0.0);
    nh_.param("x_upper_limit", x_upper_limit, 0.5);
    nh_.param("y_lower_limit", y_lower_limit, -0.5);
    nh_.param("y_upper_limit", y_upper_limit, 0.5);
    nh_.param("leg_radius", leg_radius, 0.1);
    nh_.param("min_observations", min_observations, 4);
    // nh_.param("min_predictions", min_predictions, 7);
    nh_.param("max_dist_btw_legs", max_dist_btw_legs, 1.0);
    nh_.param("z_coordinate", z_coordinate, 0.178);
    nh_.param("vel_stance_threshold", vel_stance_threshold, 0.47);
    nh_.param("vel_swing_threshold", vel_swing_threshold, 0.93);
    nh_.param("state_dimensions", state_dimensions, 6);
    nh_.param("minClusterSize", minClusterSize, 3);
    nh_.param("maxClusterSize", maxClusterSize, 100);
    nh_.param("clusterTolerance", clusterTolerance, 0.07);
    nh_.param("isOnePersonToTrack", isOnePersonToTrack, false);
    nh_.param("max_nn_gating_distance", max_nn_gating_distance, 1.0);
    nh_.param("occluded_dead_age", occluded_dead_age, 10);
    nh_.param("variance_observation", variance_observation, 0.25);
    nh_.param("min_dist_travelled", min_dist_travelled, 0.25);


    legs_gathered = id_counter = legs_marker_next_id = next_leg_id = people_marker_next_id = 0;

    sub = nh_.subscribe<sensor_msgs::LaserScan>(scan_topic, 1, &LegDetector::processLaserScan, this);
    sensor_msgs_point_cloud_publisher = nh_.advertise<sensor_msgs::PointCloud2> ("scan2cloud", 300);
    pcl_cloud_publisher = nh_.advertise<PointCloud> ("scan2pclCloud", 300);
    vis_pub = nh_.advertise<visualization_msgs::Marker>("leg_circles", 300);
    pos_vel_acc_lleg_pub = nh_.advertise<std_msgs::Float64MultiArray>("pos_vel_acc_lleg", 300);
    pos_vel_acc_rleg_pub = nh_.advertise<std_msgs::Float64MultiArray>("pos_vel_acc_rleg", 300);
    legs_and_vel_direction_publisher = nh_.advertise<visualization_msgs::MarkerArray>("legs_and_vel_direction", 300);
    tracking_area_pub = nh_.advertise<visualization_msgs::Marker>("tracking_area", 300);
    people_pub = nh_.advertise<visualization_msgs::MarkerArray>("people", 300);
//
  }

  double getRandomNumberFrom0To1()
  {
      return (double)rand() / (double)RAND_MAX ;
  }

  void handleNotSetParameter(std::string parameter)
  {
      ROS_ERROR("Parameter %s not set, shutting down node...", parameter.c_str());
      nh_.shutdown();
  }

  double calculateNorm(Point p)
  {
    return sqrt(pow(p.x, 2) + pow(p.y, 2));
  }

  int calculateGaitPhase(Point pos_left, Point pos_right, Point vel_left, Point vel_right)
  {
    double velocity_left = calculateNorm(vel_left);
    double velocity_right = calculateNorm(vel_right);

    bool isRightInStance, isLeftInStance, isRightInSwingPhase, isLeftInSwingPhase;

    if (velocity_right < velocity_left || velocity_right < vel_stance_threshold) {
      isRightInStance = true;
    }
    if (velocity_right > velocity_left || velocity_right > vel_swing_threshold) {
      isRightInSwingPhase = true;
    }
    if (velocity_left < velocity_right || velocity_left < vel_stance_threshold) {
      isRightInStance = true;
    }
    if (velocity_left > velocity_right || velocity_left > vel_swing_threshold) {
      isRightInSwingPhase = true;
    }
    // phase 0
    if (isRightInStance && isLeftInStance) { return 0; }

    double inner_pos_vel_product = (pos_left.x - pos_right.x) * vel_left.x +
				   (pos_left.y - pos_right.y) * vel_left.y;
    // phase 1
    if (isLeftInSwingPhase && isRightInStance && inner_pos_vel_product > 0) { return 1; }
    // phase 2
    if (isLeftInSwingPhase && isRightInStance && inner_pos_vel_product <= 0) { return 2; }

    inner_pos_vel_product = (pos_right.x - pos_left.x) * vel_right.x +
			    (pos_right.y - pos_left.y) * vel_right.y;
    // phase 3
    if (isRightInSwingPhase && isLeftInStance && inner_pos_vel_product > 0) { return 3; }
    // phase 4
    if (isRightInSwingPhase && isLeftInStance && inner_pos_vel_product <= 0) { return 4; }

    // phase 5
    return 5;
  }

  void considerGaitPhase() {

        /*

        unlikely:

        Phase 0 to Phase 5,
        Phase 1 to Phases 0, 3, 4 and 5,
        Phase 2 to Phases 1, 4 and 5,
        Phase 3 to Phases 0, 1, 2 and 5,
        Phase 4 to Phases 2, 3 and 5.

        */
  }


//   typedef typename Filter::MeasurementSpaceVector MeasurementSpaceVector;
//   typedef std::vector<MeasurementSpaceVector> Measurements;
//   void GNN(PointCloud measurements)
//   {
// //     template<size_t StateDim, size_t MeasurementDim>
//
// //     typedef KalmanFilter<StateDim, MeasurementDim> Filter;
//
// //     typedef std::vector<Filter> Filters;
//
//     const size_t m = measurements.points.size();
//     const size_t f = legs.size();
//
//     // create matrix for calculating distances between measurements and predictions
//     // additional rows for initializing filters (weightet by 1 / (640 * 480))
//     Eigen::MatrixXd w_ij(m, f + m);
//
//     w_ij = Eigen::MatrixXd::Zero(m, f + m);
//
//     // get likelihoods of measurements within track pdfs
//     for ( size_t i = 0; i < m; ++i )
//     {
// 	for ( size_t j = 0; j < f; ++j )
// 	{
// 	  double temp = legs[j].likelihood(measurements.points[i].x, measurements.points[i].y);
// 	  //if (!legs[j].likelihood(measurements.points[i].x, measurements.points[i].y, temp)) { continue; }
// 	  w_ij(i, j) = temp;
// 	}
//     }
//
//     // TODO: must changed to generic
//     // weights for initializing new filters
//     for ( size_t j = f; j < m + f; ++j )
// 	w_ij(j - f, j) = 1. / ((x_upper_limit - x_lower_limit) * (y_upper_limit - y_lower_limit));
//
//     // solve the maximum-sum-of-weights problem (i.e. assignment problem)
//     // in this case it is global nearest neighbour by minimizing the distances
//     // over all measurement-filter-associations
//     Auction<double>::Edges assignments = Auction<double>::solve(w_ij);
//
//     std::vector<Leg> newLegs;
//
//     // for all found assignments
//     for ( const auto & e : assignments )
//     {
// 	// is assignment an assignment from an already existing filter to a measurement?
// 	if ( e.y < f )
// 	{
// 	    // update filter and keep it
// 	    updateLeg(legs[e.y], measurements.points[e.x]);
// 	    newLegs.emplace_back(legs[e.y]);
// 	}
// 	else // is this assignment a measurement that is considered new?
// 	{
// 	    // create filter with measurment and keep it
// 	    Leg l;
// 	    if (!l.configure(state_dimensions, min_predictions, min_observations, measurements.points[e.x])) { ROS_ERROR("GNN: Configuring of leg failed!"); continue; }
// 	    newLegs.emplace_back(l);
// 	}
//     }
//
//     // current filters are now the kept filters
//     legs = newLegs;
//   }


  void pubCovarianceAsEllipse(const double meanX, const double meanY, const Eigen::MatrixXd& S)
  {

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(S);
    double l1 = solver.eigenvalues().x();
    double l2 = solver.eigenvalues().y();
    Eigen::VectorXd e1 = solver.eigenvectors().col(0);
    Eigen::VectorXd e2 = solver.eigenvectors().col(1);

    double scale95 = sqrt(5.991);
    double R1 = scale95 * sqrt(l1);
    double R2 = scale95 * sqrt(l2);
    double tilt = atan2(e2.y(), e2.x());


    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitZ());
    double orientation_w = q.w();
    double orientation_x = q.x();
    double orientation_y = q.y();
    double orientation_z = q.z();

    ROS_INFO("meanX: %f, meanY: %f, scale: %f, R1: %f, R2: %f, angle: %f",
	     meanX, meanY, scale95, R1, R2, tilt);

    //pub_oval(meanX, meanY, R1, R2, orientation_x, orientation_y, orientation_z, orientation_w, -2);
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
      transformStamped = tfBuffer.lookupTransform(transform_link, scan->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }
    tf2::doTransform(from, to, transformStamped);
    return true;
  }

  bool filterPCLPointCloud(const PointCloud& in, PointCloud& out)
  {
    if (in.points.size() < 5)
    {
      ROS_DEBUG("Filtering: Too small number of points in the input PointCloud!");
      return false;
    }

//     PointCloud::Ptr pass_through_filtered_x (new PointCloud());
//     PointCloud::Ptr pass_through_filtered_y (new PointCloud());
//     PointCloud::Ptr sor_filtered (new PointCloud());
//
    PointCloud pass_through_filtered_x;
    PointCloud pass_through_filtered_y;
//     PointCloud sor_filtered;

    pcl::PassThrough<Point> pass;
    pass.setInputCloud(in.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_lower_limit, x_upper_limit);
    pass.setFilterLimitsNegative (false);
    pass.filter ( pass_through_filtered_x );
    if ( pass_through_filtered_x.points.size() < 5)
    {
      ROS_DEBUG("Filtering: Too small number of points in the PointCloud after PassThrough filter in x direction!");
      return false;
    }
    pass.setInputCloud( pass_through_filtered_x.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_lower_limit, y_upper_limit);
    pass.filter (pass_through_filtered_y);
    if ( pass_through_filtered_y.points.size() < 5)
    {
      ROS_DEBUG("Filtering: Too small number of points in the PointCloud after PassThrough filter in y direction!");
      return false;
    }

    pcl::RadiusOutlierRemoval<Point> outrem;



    // build the filter
    outrem.setInputCloud(pass_through_filtered_y.makeShared());
    outrem.setRadiusSearch(0.02);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    outrem.filter (out);

    if (out.points.size() < 5)
    {
      ROS_DEBUG("Filtering: Too small number of points in the resulting PointCloud!");
      return false;
    }
    return true;
  }

  Leg initLeg(const Point& p)
  {
    Leg l(getNextLegId(), p, occluded_dead_age,
      variance_observation, min_observations, state_dimensions);
    return l;
  }

  // void updateLeg(Leg& l, const Point& p)
  // {
  //   ROS_INFO("updateLeg");
  //   printLegsInfo();
  //   std::vector<double> in, out;
  //   in.push_back(p.x); in.push_back(p.y);
  //   l.update(in, out);
  //   ROS_INFO("AFTER updateLeg");
  //   printLegsInfo();
  // }

  void removeLeg(int index)
  {
    ROS_INFO("removeLeg  legs[index].hasPair: %d, legs[i].getPeopleId: %d", legs[index].hasPair(), legs[index].getPeopleId());
    printLegsInfo();
    if (legs[index].hasPair())
    {
      int id = legs[index].getPeopleId();
      for (int i = 0; i < legs.size(); i++)
      {
	      if (i != index && legs[i].getPeopleId() == id)
	      {
		      legs[i].setHasPair(false);
	      }
      }
    }
    if (legs[index].getPeopleId() != -1)
	{
		//legs[index].setPeopleId(-1);
		removed_legs.push_back(legs[index]);
	}
    legs.erase(legs.begin() + index);
    ROS_INFO("AFTER removeLeg");
    printLegsInfo();
  }



/*
  void predictLeg(int i)
  {
    ROS_INFO("PredictLeg  legs[i].getPredictions: %d, legs[i].getPeopleId: %d", legs[i].getPredictions(), legs[i].getPeopleId());
    printLegsInfo();
    if (legs[i].getPredictions() >= min_predictions)
    {
      removeLeg(i);
    }
    else
    {
      legs[i].predict();
    }
    ROS_INFO("AFTER predictLeg");
    printLegsInfo();
  }*/


  void printLegsInfo()
  {
    for (Leg& l : legs)
    {
      ROS_INFO("legId: %d, peopleId: %d, pos: (%f, %f), observations: %d, hasPair: %d",
      l.getLegId(), l.getPeopleId(), l.getPos().x, l.getPos().y, l.getObservations(), l.hasPair());
    }
  }

  void visLegs(PointCloud& cloud)
  {
    cloud.points.clear();
    visualization_msgs::MarkerArray ma_leg_pos;
    visualization_msgs::MarkerArray ma_leg_vel;
    int max_id = 0;
    int id = 0;
    for (Leg& l : legs)
    {
//       ROS_INFO("VISlegs peopleId: %d, pos: (%f, %f), predictions: %d, observations: %d, hasPair: %d",
//       l.getPeopleId(), l.getPos().x, l.getPos().y, l.getPredictions(), l.getObservations(), l.hasPair());
      cloud.points.push_back(l.getPos());
      ma_leg_pos.markers.push_back(getMarker(l.getPos().x, l.getPos().y, z_coordinate, max_id/*, true*/));
//       ma_leg_vel.markers.push_back(getArrowMarker(l.getPos().x + 0.01, l.getPos().y + 0.01,
// 	  l.getPos().x + l.getVel().x + 0.01, l.getPos().y + l.getVel().y + 0.01, max_id));
      max_id++;
      id++;
    }
    legs_and_vel_direction_publisher.publish(ma_leg_pos);
//     marker_array_vel_publisher.publish(ma_leg_vel);
    pcl_cloud_publisher.publish(cloud.makeShared());
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
    if (legs_marker_next_id != 0) {
      removeOldMarkers(legs_marker_next_id, legs_and_vel_direction_publisher);
      legs_marker_next_id = 0;
    }
    visualization_msgs::MarkerArray ma_leg;
    for (Leg& l : legs)
    {
//       ROS_INFO("VISlegs peopleId: %d, pos: (%f, %f), observations: %d, hasPair: %d",
//       l.getPeopleId(), l.getPos().x, l.getPos().y, l.getObservations(), l.hasPair());

//       ma_leg.markers.push_back(getMarker(l.getPos().x, l.getPos().y, getNextLegsMarkerId()));
      if (l.getObservations() == 0 || calculateNorm(l.getVel()) < 0.01) { continue; }
      ma_leg.markers.push_back(getArrowMarker(l.getPos().x, l.getPos().y,
	       l.getPos().x + 0.5 * l.getVel().x, l.getPos().y + 0.5 * l.getVel().y, getNextLegsMarkerId()));
    }
    legs_and_vel_direction_publisher.publish(ma_leg);
    
    if (legs.size() != 2) {
      return;
    }
//     if (legs[0].getHistory().size() < 1 && legs[1].getHistory().size() < 1) { return; }
//     bool isLeft = legs[0].getPos().x < legs[1].getPos().x;
//     pub_leg_posvelacc(legs[0].getHistory().back(), isLeft);
//     pub_leg_posvelacc(legs[1].getHistory().back(), !isLeft); 
  }

  int getNextLegsMarkerId()
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
//     marker.pose.position.x = x;
//     marker.pose.position.y = y;
//     marker.pose.position.z = z_coordinate / 2;
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;

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
//     if (id == 2)
//       marker.color.g = 1.0;
//     if (id == 1)
//       marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
//     marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    return marker;
  }

  visualization_msgs::Marker getMarker(double x, double y, double scale_z, int id/*, bool isMeasurement*/)
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
//     if (!isMeasurement) { marker.pose.position.z += z_coordinate; }
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;
    marker.scale.x = leg_radius;
    marker.scale.y = leg_radius;
    marker.scale.z = scale_z;
    marker.color.a = 1.0; // Don't forget to set the alpha!
//     if (isMeasurement)
      marker.color.r = 1.0;
//     if (id == 2)
//     if (!isMeasurement)
//       marker.color.g = 1.0;
//     if (id == 1)
//       marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
//     marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    return marker;
  }

  void printClusterInfo(const PointCloud& cluster_centroids)
  {
//     ROS_INFO("Clusters:");
//     for (int i = 0; i < cluster_centroids.points.size(); i++)
//     {
//       ROS_INFO("cluster %d: (%f, %f)", i, cluster_centroids.points[i].x, cluster_centroids.points[i].y);
//     }
  }

  void matchLegCandidates(PointCloud cluster_centroids)
  {
    // printClusterInfo(cluster_centroids);
    //PointCloud predictions;
    //predictions.header = cluster_centroids.header;
    //computeKalmanFilterPredictions(predictions);
    //ROS_INFO("predictions: %d, cluster_centroids: %d", (int) predictions.points.size(), (int) cluster_centroids.points.size());
    //if (predictions.points.size() == 0 && cluster_centroids.points.size() == 0) { ROS_INFO("There are no leg candidates and no predictions!"); return; }


    // there are no measurements for legs
    // if (cluster_centroids.points.size() == 0)
    // {
    //   ROS_INFO("There are only predictions!");
    //   for (int i = 0; i < legs.size(); i++)
    //   {
    //      predictLeg(i);
    //   }
    //   return;
    // }

    //// there are no observed legs
//     if (predictions.points.size() == 0)
//     {
//       ROS_INFO("There are only new leg candidates!");
//       for (Point p : cluster_centroids.points)
//       {
// 	initLeg(p);
//       }
//       return;
//     }

//     visualization_msgs::MarkerArray ma;


    for (int i = 0; i < legs.size(); i++)
    {
      legs[i].predict();
    }


    // if there is matched measurement then update else predict
    for (int i = 0; i < legs.size(); i++)
    {
      if (cluster_centroids.points.size() == 0) { legs[i].missed();  continue; }

      Point match;
      //ma.markers.push_back(getMarker(prediction.x, prediction.y, 10, false));

//       if (!findAndEraseMatch(legs[i].getPos(), cluster_centroids, match, max_nn_gating_distance)) { legs[i].missed(); }
      if (!findAndEraseMatchWithCov(i, cluster_centroids, match)) { legs[i].missed(); }
      else { legs[i].update(match); }

      // Eigen::MatrixXd gate;
      // if (!legs[i].getGatingMatrix(gate)) { ROS_WARN("Could not get the gating matrix!"); predictLeg(i); continue; }
      //
      // pubCovarianceAsEllipse(prediction.x, prediction.y, gate);

      // if (!findAndEraseMatchWithCov(i, prediction, cluster_centroids, match)) { predictLeg(i); }
      // else { updateLeg(legs[i], match); }
    }
    cullDeadTracks(legs);

    for (Point& p : cluster_centroids.points)
    {
      legs.push_back(initLeg(p));
    }

  }

      /*
=======
//     for (int i = 0; i < legs.size(); i++)
//     {
//       if (cluster_centroids.points.size() == 0) { predictLeg(i); continue; }
//

      // NN
//       Point prediction, nearest;
//       prediction = legs[i].computePrediction();
//       ma.markers.push_back(getMarker(prediction.x, prediction.y, 10, false));
//       double radius = (2 + legs[i].getPredictions()) * leg_radius;
//       if (!findAndEraseMatch(prediction, cluster_centroids, nearest, radius)) { predictLeg(i); }
//       else { updateLeg(legs[i], nearest); }


>>>>>>> origin/master
      // GNN
      const size_t m = cluster_centroids.points.size();
      const size_t f = legs.size();

      // create matrix for calculating distances between measurements and predictions
      // additional rows for initializing filters (weightet by 1 / (640 * 480))
      Eigen::MatrixXd w_ij(m, f + m);

      w_ij = Eigen::MatrixXd::Zero(m, f + m);

      // get likelihoods of measurements within track pdfs
      for ( size_t i = 0; i < m; ++i )
      {
	  for ( size_t j = 0; j < f; ++j )
	  {
	    double temp;
	    if (!legs[j].likelihood(cluster_centroids.points[i].x, cluster_centroids.points[i].y, temp)) { continue; }
	    w_ij(i, j) = temp;
	  }
      }

      // TODO: must changed to generic
      // weights for initializing new filters
      for ( size_t j = f; j < m + f; ++j )
	  w_ij(j - f, j) = 1. / ((x_upper_limit - x_lower_limit) * (y_upper_limit - y_lower_limit));

      // solve the maximum-sum-of-weights problem (i.e. assignment problem)
      // in this case it is global nearest neighbour by minimizing the distances
      // over all measurement-filter-associations
      Auction<double>::Edges assignments = Auction<double>::solve(w_ij);


      std::vector<Leg> newLegs;

      for ( const auto & e : assignments )
      {
	  // is assignment an assignment from an already existing filter to a measurement?
	  if ( e.y < f )
	  {
	      // update filter and keep it
	      updateLeg(legs[e.y], cluster_centroids.points[e.x]);
	      newLegs.emplace_back(legs[e.y]);
	  }
	  else // is this assignment a measurement that is considered new?
	  {
	      // create filter with measurment and keep it
	      Leg l(state_dimensions, min_predictions, min_observations);
	      if (!l.configure(cluster_centroids.points[e.x])) { ROS_ERROR("GNN: Configuring of leg failed!"); continue; }
	      newLegs.emplace_back(l);
	  }
      }

      legs = newLegs;
	*/
//     }
//     legs_and_vel_direction_publisher.publish(ma);

    // if there is enough measurements for legs try to find people

    // findPeople();

    // save new measurements of legs
  //   for (Point p : cluster_centroids.points)
  //   {
  //     initLeg(p);
  //   }
  // }


  bool findAndEraseMatchWithCov(int legIndex, PointCloud& cloud, Point& out)
  {
    if (cloud.points.size() == 0) { ROS_INFO("findAndEraseMatchWithCov: Cloud is emty!"); return false; }



    int index = -1;
    double min_dist = max_cost;
    for (int i = 0; i < cloud.points.size(); i++)
    {
      double cov = legs[legIndex].getMeasToTrackMatchingCov();
      double mahalanobis_dist = std::sqrt((std::pow((cloud.points[i].x - legs[legIndex].getPos().x), 2) +
	  std::pow((cloud.points[i].y - legs[legIndex].getPos().y), 2)) / cov);
      if (mahalanobis_dist < min_dist)
      {
	index = i;
	min_dist = mahalanobis_dist;
      }
    }

    if (index == -1) { ROS_INFO("findAndEraseMatchWithCov: index = -1!"); return false; }


    out = cloud.points[index];
    cloud.points.erase(cloud.points.begin() + index);

    return true;
  }

  void separateLegs(int i, int j)
  {
    legs[i].setHasPair(false);
    legs[j].setHasPair(false);
    legs[i].setPeopleId(-1);
    legs[j].setPeopleId(-1);
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
// 	    ROS_INFO("separate leg %d and %d", i, j);
	    separateLegs(i, j);
	  }
	  break;
	}
      }
    }
  }

  Person initPerson(const Point& pos, int id)
  {
    Person p(id, pos, occluded_dead_age,
      variance_observation, min_observations);
    return p;
  }

  void findPeople(PointCloud& new_people, std::vector<int>& new_people_idx)
  {
    checkDistanceOfLegs();
    for (int i = 0; i < legs.size(); i++)
    {
      if (legs[i].hasPair() || 
	(legs[i].getPeopleId() == -1 && legs[i].getObservations() < min_observations)) 
	{ continue; }
//       if ((legs[i].getPeopleId() != -1 || legs[i].getObservations() >= min_observations) && !)
//       {
      findSecondLeg(i, new_people, new_people_idx);
//       }
    }
  }

  void findSecondLeg(int fst_leg, PointCloud& new_people, std::vector<int>& new_people_idx)
  {
//     ROS_INFO("findSecondLeg");
    //PointCloud potential_legs;
    std::vector<int> indices_of_potential_legs;
    for (int i = fst_leg + 1; i < legs.size(); i++)
    {
      if (/*i == fst_leg || */legs[i].hasPair() || legs[i].getObservations() < min_observations
	       || distanceBtwTwoPoints(legs[fst_leg].getPos(), legs[i].getPos()) > max_dist_btw_legs)
      {
	      continue;
      }

//       potential_legs.points.push_back(legs[i].getPos());
      indices_of_potential_legs.push_back(i);
    }

    if (indices_of_potential_legs.size() == 0) { ROS_DEBUG("There is no potential second leg!"); return; }

    if (indices_of_potential_legs.size() == 1) {
      setPeopleId(fst_leg, indices_of_potential_legs[0], new_people, new_people_idx);
      return;
    }
    int snd_leg = -1;
  	//snd_leg = findMatch(legs[fst_leg].getPos(), potential_legs, indices);

    double max_gain = 0.;
    for (int i = 0; i < indices_of_potential_legs.size(); i++)
    {
      double gain = 0.;
      bool isHistoryDistanceValid = true;
//       std::vector<std::vector<double> >::iterator fst_history_it = legs[fst_leg].getHistory().begin(),
//         snd_history_it = legs[indices_of_potential_legs[i]].getHistory().begin();
      int history_size = legs[fst_leg].getHistory().size();
      if (history_size != min_observations || 
	legs[indices_of_potential_legs[i]].getHistory().size() != history_size)
      {
        ROS_WARN("History check: vectors are not equal in size!");
        return;
      }
      for (int j = 0; j < history_size - 1; j++)
      {
// 	std::vector<double>& fst_history = *fst_history_it;
// 	std::vector<double>& snd_history = *snd_history_it;
	
// 	std::vector<double>::iterator fst_history = legs[fst_leg].getHistory()[j].begin();
// 	std::vector<double>::iterator snd_history = legs[indices_of_potential_legs[i]].getHistory()[j];
	
	int fst_history_size = legs[fst_leg].getHistory()[j].size();
	int snd_history_size = legs[indices_of_potential_legs[i]].getHistory()[j].size();
	
// 	ROS_WARN("fst_history: %d, snd_history: %d", fst_history_fst_value, snd_history_size);
	
	if (fst_history_size != state_dimensions || snd_history_size != state_dimensions)
	{
	  ROS_WARN("History check: stae vectors are not valid!");
	  return;
	}
	
// 	std::string s = "";
// 	for (int k = 0; k <= history_size; k++) {
// 	  for (double d : fst_history) {
// 	    s += std::to_string(d); s += " ";
// 	  }
// 	  s += "\n";
// 	}
// 	ROS_WARN("fst_history: \n%s", s.c_str());
// 	s = "";
// 	for (int k = 0; k <= history_size; k++) {
// 	  for (double d : snd_history) {
// 	    s += std::to_string(d); s += " ";
// 	  }
// 	  s += "\n";
// 	}
// 	ROS_WARN("snd_history: \n%s", s.c_str());
	
	double dist = distanceBtwTwoPoints(legs[fst_leg].getHistory()[j][0], 
					   legs[fst_leg].getHistory()[j][1],
	  legs[indices_of_potential_legs[i]].getHistory()[j][0], 
	  legs[indices_of_potential_legs[i]].getHistory()[j][1]);
	
	ROS_WARN("dist btw (%f, %f) and (%f, %f): %f, ids: %d %d", legs[fst_leg].getHistory()[j][0], 
					   legs[fst_leg].getHistory()[j][1],
	  legs[indices_of_potential_legs[i]].getHistory()[j][0], 
	  legs[indices_of_potential_legs[i]].getHistory()[j][1],
		 dist, legs[fst_leg].getLegId(), legs[indices_of_potential_legs[i]].getLegId());
	
	if (dist > max_dist_btw_legs)
	{
	  ROS_DEBUG("History check: distance is not valid!");
	  isHistoryDistanceValid = false;
	  break;
	}
      
//       	if (distanceBtwTwoPoints(fst_history_it->at(0), fst_history_it->at(1),
//       	  snd_history_it->at(0), snd_history_it->at(1)) > max_dist_btw_legs)
//       	{
//       	  ROS_DEBUG("History check: distance is not valid!");
//       	  isHistoryDistanceValid = false;
//       	  break;
//       	}

//         double dist = distanceBtwTwoPoints(fst_history_it->at(0), fst_history_it->at(1),
//       	  snd_history_it->at(0), snd_history_it->at(1));
        double forgettingFactor = std::pow(0.5, history_size - 1 - j);
        gain += forgettingFactor * (1 - dist / std::sqrt(200));

//       	fst_history_it++; snd_history_it++;

      }

      gain /= history_size;

      // if (isHistoryDistanceValid) { snd_leg = i; break; }
      if (!isHistoryDistanceValid) { continue; }
      if (max_gain < gain) {
        max_gain = gain;
        snd_leg = indices_of_potential_legs[i];
      }
    }

    if (snd_leg == -1) { ROS_INFO("Could not find second leg!"); return; }

    setPeopleId(fst_leg, snd_leg, new_people, new_people_idx);
  }

  double distanceBtwTwoPoints(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
  }


  /*int findMatch(const Point& searchPoint, const PointCloud& cloud, const std::vector<int>& indices)
  {
    ROS_INFO("findMatch for (%f, %f) with radius %f", searchPoint.x, searchPoint.y, radius_of_person);
    printCloudPoints(cloud);
    int out_index = -1;
    if (cloud.points.size() == 0) { return out_index; }
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud (cloud.makeShared());
    std::vector<int> pointIdxRadius;
    std::vector<float> pointsSquaredDistRadius;
    int count = kdtree.radiusSearch (searchPoint, radius_of_person, pointIdxRadius, pointsSquaredDistRadius);

    if (count == 0) { return out_index; }
//     else if (count == 1)
//     {
      int K = 1;
      std::vector<int> pointsIdx(K);
      std::vector<float> pointsSquaredDist(K);
      kdtree.nearestKSearch (searchPoint, K, pointsIdx, pointsSquaredDist);
      ROS_INFO("legs.size: %d, pointsIdx[0]: %d, indices[pointsIdx[0]]: %d, pointsSquaredDist: %f",
	(int) legs.size(), pointsIdx[0], indices[pointsIdx[0]], pointsSquaredDist[0]);
      if (pointsSquaredDist[0] <= radius_of_person && pointsSquaredDist[0] >= 0.5 * leg_radius) {
	out_index = indices[pointsIdx[0]];
      }
//     }
//     else //(count > 1)
//     {
//       ROS_INFO("mahalanobis");
//       // interesting more than one leg matched
//       // use mahalanobis distance and some other data association methods
//
//
//       int index = -1;
//       double max_likelihood = -1.;
//       for (int i = 0; i < cloud.points.size(); i++)
//       {
// 	double likelihood;
// 	if (!legs[legIndex].likelihood(cloud.points[i].x, cloud.points[i].y, likelihood)) { return out_index; }
// 	ROS_INFO("legIndex: %d, sPoint.x: %f, sPoint.y: %f, max_likelihood: %f, cloudPoint[%d].x: %f, cloudPoint[%d].y: %f, likelihood: %f",
// 	  legIndex, searchPoint.x, searchPoint.y, max_likelihood, i, cloud.points[i].x, i, cloud.points[i].y, likelihood);
// 	if (likelihood > max_likelihood)
// 	{
// 	  index = i;
// 	  max_likelihood = likelihood;
// 	}
//       }
//
//       if (index == -1) { return out_index; }
//     }

    return out_index;
  }*/


  void setPeopleId(int fst_leg, int snd_leg, PointCloud& new_people, std::vector<int>& new_people_idx)
  {
	  int id = -1;
	  bool isIdSet = false;
	  isIdSet = legs[fst_leg].getPeopleId() != -1 || legs[snd_leg].getPeopleId() != -1;
	  if (isIdSet && (legs[snd_leg].getPeopleId() ==  -1 || legs[snd_leg].getPeopleId() == legs[fst_leg].getPeopleId()) )
// 	  if (isIdSet && legs[fst_leg].getPeopleId() !=  -1)
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
		  // legs from not same people
		  //return;
			id = std::min(legs[fst_leg].getPeopleId(), legs[snd_leg].getPeopleId());

			eraseRemovedLeg(legs[fst_leg].getPeopleId());
			eraseRemovedLeg(legs[snd_leg].getPeopleId());
	  }
	  else
	  {
		  id = id_counter++;
	  }




	  legs[fst_leg].setPeopleId(id);
	  legs[snd_leg].setPeopleId(id);
	  legs[fst_leg].setHasPair(true);
	  legs[snd_leg].setHasPair(true);

// 	  Point person;
// 	  person.x = (legs[fst_leg].getPos().x + legs[snd_leg].getPos().x) / 2;
// 	  person.y = (legs[fst_leg].getPos().y + legs[snd_leg].getPos().y) / 2;

// 	  new_people.points.push_back(person);
// 	  new_people_idx.push_back(id);

	  //std::pair<PeopleMap::iterator, bool> ret;
	  //std::pair <Point, Point> leg_points = std::make_pair(legs[fst_leg].getPos(), legs[fst_leg].getPos());
	  //ret = persons.insert ( std::pair<int, std::pair <Point, Point> >(id, leg_points) );
	  //if (ret.second==false) {
		//std::cout << "element 'z' already existed";
		//std::cout << " with a value of " << ret.first->second << '\n';
	  //}
	  //persons.add(id, std::make_pair(legs[fst_leg].getPos(), legs[fst_leg].getPos()));
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

  void printCloudPoints(const PointCloud& cloud)
  {
    for (int i = 0; i < cloud.points.size(); i++)
    {
      ROS_INFO("Point %d: (%f, %f)", i, cloud.points[i].x, cloud.points[i].y);
    }
  }

  double distanceBtwTwoPoints(Point p1, Point p2)
  {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
  }


  void calculateAndPubOval(double x1, double y1, double x2, double y2, int id)
  {
	    double x = (x1 + x2) / 2;
	    double y = (y1 + y2) / 2;
	    //double dist = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
      double dist = distanceBtwTwoPoints(x1, y1, x2, y2);
	    double scale_x = dist + 5 * leg_radius;
	    double scale_y = 5 * leg_radius;
// 	    ROS_INFO("x1 - x2: %f", std::abs(x1 - x2));

	    double norm_1 = std::sqrt(std::pow(x1, 2) + std::pow(y1, 2));
	    double norm_2 = std::sqrt(std::pow(x2, 2) + std::pow(y2, 2));
	    double temp = (x1 * x2 + y1 * y2) / (norm_1 * norm_2);


	    double diff_x = x1 - x2;
// 	    if (x1 > x2) { diff_x = x2 - x1; }
	    double diff_y = y1 - y2;
// 	    ROS_INFO("angle_x: %f, angle_y: %f", diff_x, diff_y);

	    double angle = std::atan2( diff_y, diff_x );
// 	    ROS_WARN("diff_y: %f, diff_x : %f, angle: %f", diff_y, diff_x, angle);
// 	    double angle = std::acos(temp);


	    Eigen::Quaterniond q;
	    q = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
	    double orientation_w = q.w();
	    double orientation_x = q.x();
	    double orientation_y = q.y();
	    double orientation_z = q.z();

	    /*
	    ROS_INFO("Oval theta: %f, x: %f, y: %f, s_x: %f, s_y: %f, o_x: %f, o_y: %f, o_z: %f, o_w: %f, id: %d",
	      angle, x, y, scale_x, scale_y, orientation_x, orientation_y, orientation_z, orientation_w, id);*/
	    pub_oval(x, y, scale_x, scale_y, orientation_x, orientation_y, orientation_z, orientation_w, id);
  }

  visualization_msgs::Marker getOvalMarker(double x1, double y1, double x2, double y2, int id)
  {
    double x = (x1 + x2) / 2;
    double y = (y1 + y2) / 2;
    double dist = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    double scale_x = dist + 5 * leg_radius;
    double scale_y = 5 * leg_radius;
// 	    ROS_INFO("x1 - x2: %f", std::abs(x1 - x2));

    double norm_1 = std::sqrt(std::pow(x1, 2) + std::pow(y1, 2));
    double norm_2 = std::sqrt(std::pow(x2, 2) + std::pow(y2, 2));
    double temp = (x1 * x2 + y1 * y2) / (norm_1 * norm_2);


    double diff_x = x1 - x2;
// 	    if (x1 > x2) { diff_x = x2 - x1; }
    double diff_y = y1 - y2;
// 	    ROS_INFO("angle_x: %f, angle_y: %f", diff_x, diff_y);

    double angle = std::atan2( diff_y, diff_x );
// 	    ROS_WARN("diff_y: %f, diff_x : %f, angle: %f", diff_y, diff_x, angle);
// 	    double angle = std::acos(temp);

    Eigen::Quaterniond q;

    q = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    double orientation_w = q.w();
    double orientation_x = q.x();
    double orientation_y = q.y();
    double orientation_z = q.z();

    /*
    ROS_INFO("Oval theta: %f, x: %f, y: %f, s_x: %f, s_y: %f, o_x: %f, o_y: %f, o_z: %f, o_w: %f, id: %d",
      angle, x, y, scale_x, scale_y, orientation_x, orientation_y, orientation_z, orientation_w, id);*/
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    if (id == -2) { marker.id = 0; }
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
//     if (id == 0)
//       marker.color.r = 1.0;
//     if (id == 2)
//       marker.color.g = 1.0;
    if (id == -2) { marker.color.b = 1.0; }
//     if (id == 1)
//       marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
//     marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    return marker;
  }

  void vis_persons()
  {
    if (people_marker_next_id != 0) {
      removeOldMarkers(people_marker_next_id, people_pub);
      people_marker_next_id = 0;
    }

    visualization_msgs::MarkerArray ma_people;

    for (int i = 0; i < persons.size(); i++) {
	    ma_people.markers.push_back(getMarker(persons[i].getPos().x,
		persons[i].getPos().y, z_coordinate * 3, getPeopleMarkerNextId()));
    }


    people_pub.publish(ma_people);
  }


  void vis_people()
  {
    if (people_marker_next_id != 0) {
      removeOldMarkers(people_marker_next_id, people_pub);
      people_marker_next_id = 0;
    }
    visualization_msgs::MarkerArray ma_people;
//     int max_id = 0;

    for (int i = 0; i < legs.size(); i++)
    {
      int id = legs[i].getPeopleId();
      if (id == -1) { continue; }
      // if (legs[i].getDistTravelled() < min_dist_travelled) {
      //   ROS_DEBUG("Distance travelled: not enough!");
      //   continue;
      // }

	  // second leg is removed
	if (!legs[i].hasPair())
	{
		for (Leg& l : removed_legs)
		{
			if (l.getPeopleId() == id)
			{
			  ma_people.markers.push_back(getOvalMarker(legs[i].getPos().x,
				legs[i].getPos().y, l.getPos().x, l.getPos().y, getPeopleMarkerNextId()));
			  /*
			  updatePersonList(id, (legs[i].getPos().x + l.getPos().x) / 2, 
					   (legs[i].getPos().y + l.getPos().y) / 2);*/
// 			  max_id++;

// 			  ROS_INFO("VISpeople peopleId: %d, pos1: (%f, %f), pos2removed: (%f, %f), predictions: (%d, %d), observations: (%d, %d), hasPair: (%d, %d)",
// 			  id, legs[i].getPos().x, legs[i].getPos().y, l.getPos().x, l.getPos().y, legs[i].getPredictions(), l.getPredictions(),
// 				   legs[i].getObservations(), l.getObservations(), legs[i].hasPair(), l.hasPair());

			  break;
			}
		}

	}

      if (legs[i].hasPair())
      {
	for (int j = i + 1; j < legs.size(); j++)
	{
	  if (legs[j].getPeopleId() == id)
	  {
	    ma_people.markers.push_back(getOvalMarker(legs[i].getPos().x,
		legs[i].getPos().y, legs[j].getPos().x, legs[j].getPos().y, getPeopleMarkerNextId()));
	    /*
	    updatePersonList(id, (legs[i].getPos().x + l.getPos().x) / 2, 
		(legs[i].getPos().y + l.getPos().y) / 2);*/
// 	    max_id++;

// 	    ROS_INFO("VISpeople peopleId: %d, pos1: (%f, %f), pos2: (%f, %f), predictions: (%d, %d), observations: (%d, %d), hasPair: (%d, %d)",
// 	    id, legs[i].getPos().x, legs[i].getPos().y,
// 		     legs[j].getPos().x, legs[j].getPos().y,
// 		     legs[i].getPredictions(), legs[j].getPredictions(),
// 		      legs[i].getObservations(), legs[j].getObservations(),
// 		     legs[i].hasPair(), legs[j].hasPair());
	  }
	}
      }
    }

    people_pub.publish(ma_people);
  }
  /*
  void updatePersonList(int id, double x, double y)
  {
    
  }*/

  unsigned int getPeopleMarkerNextId() {
    return people_marker_next_id++;
  }

  bool findAndEraseMatch(const Point& searchPoint, PointCloud& cloud, Point& out, double radius)
  {
    if (cloud.points.size() == 0) { return false; }
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud (cloud.makeShared());
    // std::vector<int> pointIdxRadius;
    // std::vector<float> pointsSquaredDistRadius;
    // // radius search
    // int count = kdtree.radiusSearch (searchPoint, radius, pointIdxRadius, pointsSquaredDistRadius);
    // if (count == 0) { return false; }
    // if (count > 1)
    // {
    //   //interesting more than one point matched
    // }
    int K = 1;
    std::vector<int> pointsIdx(K);
    std::vector<float> pointsSquaredDist(K);
    kdtree.nearestKSearch (searchPoint, K, pointsIdx, pointsSquaredDist);
    if (pointsSquaredDist[0] > radius && pointsSquaredDist[0] < 0.5 * leg_radius) {
      return false;
    }
    out = cloud.points[pointsIdx[0]];
    cloud.points.erase(cloud.points.begin() + pointsIdx[0]);
    return true;
  }



  void computeKalmanFilterPredictions(PointCloud& predictions)
  {
//     for (Leg& l : legs)
//     {
//       predictions.points.push_back(l.computePrediction());
//     }
  }

  bool clustering(const PointCloud& cloud, PointCloud& cluster_centroids)
  {

    if (cloud.points.size() < minClusterSize) { ROS_DEBUG("Clustering: Too small number of points!"); return false; }
//     pcl::PCDWriter writer;

    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud (cloud.makeShared());
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance (clusterTolerance); // 4cm
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud.makeShared());
    ec.extract (cluster_indices);

    cluster_centroids.header = cloud.header;
    cluster_centroids.points.clear();

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<Point>::Ptr cloud_cluster (new pcl::PointCloud<Point>);
      cloud_cluster->header = cloud.header;
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	cloud_cluster->points.push_back (cloud.points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;


//       std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;


      std::vector<double> c;
      computeCircularity(cloud_cluster, c);

      if (c.size() == 2)
      {
	Point p; p.x = c[0]; p.y = c[1];
	cluster_centroids.points.push_back(p);
// 	pub_circle_with_id(c[0], c[1], j);
      }
//       std::stringstream ss;
//       ss << "cloud_cluster_" << j << ".pcd";
//       writer.write<Point> (ss.str (), *cloud_cluster, false); //*
      j++;
    }
//     pcl_cloud_publisher.publish(cluster_centroids);

 //   std::cout << "clusters: " << j << " datas." << std::endl;
    return true;
  }

  /*void sortPointCloudToLeftAndRight(const PointCloud& input_cloud, PointCloud::Ptr left, PointCloud::Ptr right)
  {

//     #if CV_MAJOR_VERSION == 2
    // do opencv 2 code


    // convert input cloud to opencv Mat
    cv::Mat points(input_cloud.size(),1, CV_32FC3);
    std::size_t idx = 0;

//     std::cout << "Input model points:\n";
    for(PointCloud::const_iterator it = input_cloud.begin(); it != input_cloud.end(); ++it, ++idx)
    {
	    points.at<cv::Vec3f>(idx,0) = cv::Vec3f(it->x,it->y);
	    // std::cerr << points.at<cv::Vec3f>(0,idx) << std::endl;
    }

    // reshape to

    cv::Mat labels;
    // cv::Mat(sampleCount,1,CV_32S);


    int attempts = 10;
    cv::Mat centers;

    int max_cluster_size = input_cloud.size() > cluster_size ? cluster_size : input_cloud.size();

    // use opencv kmeans to extract the cluster center, since pcl 1.7.2 does not have kmeans
    cv::kmeans(points, max_cluster_size, labels,
	    cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10e4, 1e-4),
	    attempts, cv::KMEANS_RANDOM_CENTERS, centers);

    if (centers.rows != 2)
    {
      ROS_INFO("KMeans: The number of rows is not valid!");
      return;
    }


    cv::Vec3f cv_center1 = centers.at<cv::Vec3f>(0);
    cv::Vec3f cv_center2 = centers.at<cv::Vec3f>(1);

    cv::Point2f c1(cv_center1[0], cv_center1[1]);
    cv::Point2f c2(cv_center2[0], cv_center2[1]);

    double dist_to_center1 = cv::norm(left_leg_prediction - c1);
    double dist_to_center2 = cv::norm(left_leg_prediction - c2);

    int leftId = dist_to_center1 < dist_to_center2 ? 0 : 1;

    // compare two centers
    // cv_center1[0],cv_center1[1] && cv_center2[0],cv_center2[1]

    // for example
    // is y of the first center bigger than y of the second center?
//     int leftId = cv_center1[1] > cv_center2[1] ? 0 : 1;
//     ROS_INFO("0: %f, 1: %f, leftId: %d", cv_center1[1], cv_center2[1], leftId);

    int i = 0;
    for(PointCloud::const_iterator it = input_cloud.begin(); it != input_cloud.end(); ++it, ++i)
    {
      int id = labels.at<int>(i,0);
      if (id == leftId)
      {
	left->points.push_back(*it);
      }
      else
      {
	right->points.push_back(*it);
      }
    }

//     #elif CV_MAJOR_VERSION == 3
    // do opencv 3 code
//     #endif
  }*/

//   std::vector<double> computeCentroids(PointCloud::Ptr cloud)
//   {
//     std::vector<double> result;
//     Eigen::Vector4f centroid;
//     pcl::compute2DCentroid (*cloud, centroid);
//     result.push_back(centroid(0));
//     result.push_back(centroid(1));
//     return result;
//   }

  std::vector<double> computeRansacPubInliersAndGetCenters(PointCloud::Ptr cloud)
  {
    std::vector<double> centers;
    PointCloud toPub;
    toPub.header = cloud->header;

    pcl::SampleConsensusModelCircle2D<Point>::Ptr model(
      new pcl::SampleConsensusModelCircle2D<Point> (cloud));

    pcl::RandomSampleConsensus<Point> ransac (model);

    ransac.setDistanceThreshold (ransac_dist_threshold);

    try
    {
      ransac.computeModel();
    }
    catch (...)
    {
      ROS_INFO("Ransac: Computing model has failed!");
      return centers;
    }

    /*
     * get the radius of the circles
     */
    Eigen::VectorXf circle_coeff;

    std::vector<int> inliers;
    ransac.getInliers(inliers);
    if (inliers.size() < 3)
    {
      ROS_INFO("The number of inliers is too small!");
      return centers;
    }

    for (int i : inliers)
    {
      toPub.points.push_back(cloud->points[i]);
    }
    ransac.getModelCoefficients(circle_coeff);

    centers.push_back(circle_coeff(1));
    centers.push_back(circle_coeff(2));


    Point point;
    point.x = circle_coeff(0);
    point.y = circle_coeff(1);
    toPub.points.push_back(point);

    pcl_cloud_publisher.publish(toPub.makeShared());

    return centers;
  }

  void pub_leg_posvelacc(std::vector<double>& in, bool isLeft)
  {
    if (in.size() != 6) { ROS_ERROR("Invalid vector of leg posvelacc!"); return; }

    std_msgs::Float64MultiArray msg;

    // set up dimensions
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = in.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "pos_vel_acc";

    // copy in the data
    msg.data.clear();
    msg.data.insert(msg.data.end(), in.begin(), in.end());

    if (isLeft) { pos_vel_acc_lleg_pub.publish(msg); }
    else { pos_vel_acc_rleg_pub.publish(msg); }
  }

//   bool useKalmanFilterAndPubCircles(const PointCloud::Ptr cloud, bool isLeft)
//   {
//     const int num_elements_measurement = 2;
//     const int num_elements_kalman = 6;
//     std::vector<double> centerOfLegMeasurement;
//     std::vector<double> centerOfLegKalman;
// //     if (circle_fitting == "ransac")
// //     {
// //       centerOfLegMeasurement = computeRansacPubInliersAndGetCenters(cloud);
// //     }
// //     else
// //     {
// //       centerOfLegMeasurement = computeCentroids(cloud);
// //     }
//     if (!computeCircularity(cloud, centerOfLegMeasurement)) { return false; }
// 
//     if (centerOfLegMeasurement.size() < num_elements_measurement)
//     {
//       ROS_ERROR("Center (measurement) does not have enough elements!");
//       return false;
//     }
// 
// //     if (abs(centerOfLegMeasurement[0] - centerOfLegLastMeasurement[0]) > ...)
// //     {
// //
// //     }
//     std::vector<double> prediction;
//     if (isLeft) { left_leg_filter->predict(prediction); left_leg_prediction = cv::Point2f(prediction[0], prediction[1]); }
//     else { right_leg_filter->predict(prediction); right_leg_prediction = cv::Point2f(prediction[0], prediction[1]); }
//     pub_circle(prediction[0], prediction[1], leg_radius, isLeft, isLeft ? 2 : 3);
// 
//     if (isLeft) { left_leg_filter->update(centerOfLegMeasurement, centerOfLegKalman); }
//     else { right_leg_filter->update(centerOfLegMeasurement, centerOfLegKalman); }
// 
// //     pub_circle(centerOfLegMeasurement[0], centerOfLegMeasurement[1], leg_radius, isLeft, isLeft ? 0 : 1);
// 
// 
// 
//     if (centerOfLegKalman.size() < num_elements_kalman)
//     {
//       ROS_ERROR("Centers (kalman) do not have enough elements!");
//       return false;
//     }
// 
// //     pub_circle(centerOfLegKalman[0], centerOfLegKalman[1], leg_radius, isLeft, isLeft ? 2 : 3);
//     if (legs_gathered == 1) {
//       person_center.x = (person_center.x + centerOfLegKalman[0]) / 2;
//       person_center.y = (person_center.y + centerOfLegKalman[1]) / 2;
//       legs_gathered++;
//     } else {
//       if (legs_gathered == 2) {
// 	pub_circle(person_center.x, person_center.y, 0.7, isLeft, 0);
//       }
//       person_center.x = centerOfLegKalman[0];
//       person_center.y = centerOfLegKalman[1];
//       legs_gathered = 1;
//     }
// 
//     pub_leg_posvelacc(centerOfLegKalman, isLeft);
// 
//     return true;
//   }

  bool computeCircularity(const PointCloud::Ptr cloud, std::vector<double>& center)
  {
    int num_points = cloud->points.size();
    if (num_points < minClusterSize) { ROS_ERROR("Circularity and Linerity: Too small number of points!"); return false; }
    double x_mean, y_mean;
    CvMat* A = cvCreateMat(num_points, 3, CV_64FC1);
    CvMat* B = cvCreateMat(num_points, 1, CV_64FC1);

//     CvMat* points = cvCreateMat(num_points, 2, CV_64FC1);

    int j = 0;
    for (Point p : cloud->points)
    {
      x_mean += p.x / num_points;
      y_mean += p.y / num_points;

//       cvmSet(points, j, 0, p.x - x_mean);
//       cvmSet(points, j, 1, p.y - y_mean);

      cvmSet(A, j, 0, -2.0 * p.x);
      cvmSet(A, j, 1, -2.0 * p.y);
      cvmSet(A, j, 2, 1);

      cvmSet(B, j, 0, -pow(p.x, 2) - pow(p.y, 2));
      j++;
    }

//     CvMat* W = cvCreateMat(2, 2, CV_64FC1);
//     CvMat* U = cvCreateMat(num_points, 2, CV_64FC1);
//     CvMat* V = cvCreateMat(2, 2, CV_64FC1);
//     cvSVD(points, W, U, V);
//
//     CvMat* rot_points = cvCreateMat(num_points, 2, CV_64FC1);
//     cvMatMul(U, W, rot_points);
//
//     // Compute Linearity
//     double linearity = 0.0;
//     for (int i = 0; i < num_points; i++)
//     {
//       linearity += pow(cvmGet(rot_points, i, 1), 2);
//     }
//
//     cvReleaseMat(&points);
//     points = 0;
//     cvReleaseMat(&W);
//     W = 0;
//     cvReleaseMat(&U);
//     U = 0;
//     cvReleaseMat(&V);
//     V = 0;
//     cvReleaseMat(&rot_points);
//     rot_points = 0;



    // Compute Circularity

    CvMat* sol = cvCreateMat(3, 1, CV_64FC1);

    cvSolve(A, B, sol, CV_SVD);

    double xc = cvmGet(sol, 0, 0);
    double yc = cvmGet(sol, 1, 0);
    double rc = sqrt(pow(xc, 2) + pow(yc, 2) - cvmGet(sol, 2, 0));

    center.clear();
    center.push_back(xc);
    center.push_back(yc);

    return true;

//     pub_circle(xc, yc, rc, isLeft, isLeft ? 0 : 1);

//     cvReleaseMat(&A);
//     A = 0;
//     cvReleaseMat(&B);
//     B = 0;
//     cvReleaseMat(&sol);
//     sol = 0;
//
//     float circularity = 0.0;
//     for (SampleSet::iterator i = cluster->begin();
// 	i != cluster->end();
// 	i++)
//     {
//       circularity += pow(rc - sqrt(pow(xc - (*i)->x, 2) + pow(yc - (*i)->y, 2)), 2);
//     }

  }

   void predictLegs()
   {
     for (Leg& l : legs) {
       l.predict();
     }
     cullDeadTracks(legs);
   }

   void removeLegFromVector(std::vector<Leg>& v, int i)
   {
      if (i != v.size() - 1) {
	Leg l = v[v.size() - 1];
	v[i] = l;
      }
      v.pop_back();
   }

   void removePersonFromVector(std::vector<Person>& v, int i)
   {
      if (i != v.size() - 1) {
	Person p = v[v.size() - 1];
	v[i] = p;
      }
      v.pop_back();
   }

   void resetHasPair(std::vector<Leg>& v, int fst_leg)
   {
      for (int snd_leg = 0; snd_leg < v.size(); snd_leg++) {
	if (snd_leg != fst_leg && v[snd_leg].getPeopleId() == v[fst_leg].getPeopleId()) {
	  ROS_WARN("heir fst: %d, snd: %d", fst_leg, snd_leg);
	  v[snd_leg].setHasPair(false);
	  break;
	}
      }
   }

   void cullDeadTracks(std::vector<Leg>& v)
   {
     int i = 0;
     while(i != v.size()) {
        if (v[i].is_dead()) {
	    if (v[i].hasPair()) { resetHasPair(v, i); removed_legs.push_back(v[i]); }
	    else if (v[i].getPeopleId() != -1) { removed_legs.push_back(v[i]); }
	    removeLegFromVector(v, i);
        } else {
             i++;
        }
     }
   }

   void cullDeadTracksOfPersons(std::vector<Person>& v)
   {
     int i = 0;
     while(i != v.size()) {
        if (v[i].is_dead()) {
	    removePersonFromVector(v, i);
        } else {
             i++;
        }
     }
   }


   void gnn_munkres_people(const PointCloud& new_people, std::vector<int>& new_people_idx)
   {
      // Filter model predictions
      for(std::vector<Person>::iterator it = persons.begin();
          it != persons.end(); it++) {
           it->predict();
      }
      std::vector<Person> fused;

      assign_munkres_people(new_people, persons, fused, new_people_idx);

      cullDeadTracksOfPersons(fused);

      persons = fused;
   }

   void assign_munkres_people(const PointCloud& new_people,
                         std::vector<Person> &tracks,
                         std::vector<Person> &fused, std::vector<int>& new_people_idx)
   {
     // Create cost matrix between previous and current blob centroids
       int meas_count = new_people.points.size();
       int tracks_count = tracks.size();

       // Determine max of meas_count and tracks
       int rows = -1, cols = -1;
       if (meas_count >= tracks_count) {
            rows = cols = meas_count;
       } else {
            rows = cols = tracks_count;
       }

       Matrix<double> matrix(rows, cols);

       // New measurements are along the Y-axis (left hand side)
       // Previous tracks are along x-axis (top-side)
       int r = 0;
       for(const Point& p : new_people.points) {
            std::vector<Person>::iterator it_prev = tracks.begin();
            int c = 0;
            for (; it_prev != tracks.end(); it_prev++, c++) {
              // if (it_prev->isPerson() and not it_prev->isInFreeSpace()) {
              //   matrix(r,c) = max_cost;
              // } else {
                double cov = it_prev->getMeasToTrackMatchingCov();
                double mahalanobis_dist = std::sqrt((std::pow((p.x - it_prev->getPos().x), 2) +
                                                     std::pow((p.y - it_prev->getPos().y), 2)) / cov);
                if (mahalanobis_dist < 2 * mahalanobis_dist_gate) {
                  matrix(r,c) = mahalanobis_dist;
                } else {
                  matrix(r,c) = max_cost;
                }
              // }
            }
            r++;
       }

       Munkres<double> m;
       m.solve(matrix);

       // Use the assignment to update the old tracks with new blob measurement
       int meas_it = 0;
       for(r = 0; r < rows; r++) {
            std::vector<Person>::iterator it_prev = tracks.begin();
            for (int c = 0; c < cols; c++) {
                 if (matrix(r,c) == 0) {
                      if (r < meas_count && c < tracks_count) {
                           // Does the measurement fall within 3 std's of
                           // the track?
                           if (it_prev->is_within_region(new_people.points[meas_it], 3)) {
                                // Found an assignment. Update the new measurement
                                // with the track ID and age of older track. Add
                                // to fused list
                                //it->matched_track(*it_prev);
                                it_prev->update(new_people.points[meas_it]);
                                fused.push_back(*it_prev);
                           } else {
                                // TOO MUCH OF A JUMP IN POSITION
                                // Probably a missed track or a new track
                                it_prev->missed();
                                fused.push_back(*it_prev);

                                // And a new track
                                fused.push_back(initPerson(new_people.points[meas_it], new_people_idx[meas_it]));
                           }
                      } else if (r >= meas_count) {
                           it_prev->missed();
                           fused.push_back(*it_prev);
                      } else if (c >= tracks_count) {
                           // Possible new track
                           fused.push_back(initPerson(new_people.points[meas_it], new_people_idx[meas_it]));
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

   void gnn_munkres(PointCloud& cluster_centroids)
   {
      // Filter model predictions
      for(std::vector<Leg>::iterator it = legs.begin();
          it != legs.end(); it++) {
           it->predict();
      }
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

       // New measurements are along the Y-axis (left hand side)
       // Previous tracks are along x-axis (top-side)
       int r = 0;
       for(const Point& p : meas.points) {
            std::vector<Leg>::iterator it_prev = tracks.begin();
            int c = 0;
            for (; it_prev != tracks.end(); it_prev++, c++) {
              // if (it_prev->isPerson() and not it_prev->isInFreeSpace()) {
              //   matrix(r,c) = max_cost;
              // } else {
                double cov = it_prev->getMeasToTrackMatchingCov();
                double mahalanobis_dist = std::sqrt((std::pow((p.x - it_prev->getPos().x), 2) +
                                                     std::pow((p.y - it_prev->getPos().y), 2)) / cov);
                if (mahalanobis_dist < mahalanobis_dist_gate) {
                  matrix(r,c) = mahalanobis_dist;
                } else {
                  matrix(r,c) = max_cost;
                }
              // }
            }
            r++;
       }

       Munkres<double> m;
       m.solve(matrix);

       // Use the assignment to update the old tracks with new blob measurement
       int meas_it = 0;
       for(r = 0; r < rows; r++) {
            std::vector<Leg>::iterator it_prev = tracks.begin();
            for (int c = 0; c < cols; c++) {
                 if (matrix(r,c) == 0) {
                      if (r < meas_count && c < tracks_count) {
                           // Does the measurement fall within 3 std's of
                           // the track?
                           if (it_prev->is_within_region(meas.points[meas_it],3)) {
                                // Found an assignment. Update the new measurement
                                // with the track ID and age of older track. Add
                                // to fused list
                                //it->matched_track(*it_prev);
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

  void pub_circle_with_id(double x, double y, int id)
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
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;
    marker.scale.x = leg_radius;
    marker.scale.y = leg_radius;
//     marker.scale.z = 0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    if (id == 0)
      marker.color.r = 1.0;
    if (id == 2)
      marker.color.g = 1.0;
    if (id == 1)
      marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
//     marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub.publish( marker );

  }

  void pub_line(double x, double y, double width)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
//     marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
//     marker.pose.orientation.x = orientation_x;
//     marker.pose.orientation.y = orientation_y;
//     marker.pose.orientation.z = orientation_z;
//     marker.pose.orientation.w = orientation_w;
    marker.scale.x = width;
//     marker.scale.y = y_upper_limit - y_lower_limit;
//     marker.scale.z = 0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.g = 1.0;
//     if (id == 0)
//       marker.color.r = 1.0;
//     if (id == 2)
//       marker.color.g = 1.0;
//     if (id == 1)
//       marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
//     marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub.publish( marker );
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
    marker.pose.position.x = (x_upper_limit + x_lower_limit) / 2;
    marker.pose.position.y = (y_upper_limit + y_lower_limit) / 2;
    marker.pose.position.z = 0.0;
//     marker.pose.orientation.x = orientation_x;
//     marker.pose.orientation.y = orientation_y;
//     marker.pose.orientation.z = orientation_z;
//     marker.pose.orientation.w = orientation_w;
    marker.scale.x = x_upper_limit - x_lower_limit;
    marker.scale.y = y_upper_limit - y_lower_limit;
//     marker.scale.z = 0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.g = 1.0;
//     if (id == 0)
//       marker.color.r = 1.0;
//     if (id == 2)
//       marker.color.g = 1.0;
//     if (id == 1)
//       marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
//     marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    tracking_area_pub.publish( marker );
  }

  void pub_oval(double x, double y, double scale_x, double scale_y, double orientation_x,
    double orientation_y, double orientation_z, double orientation_w, int id)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    if (id == -2) { marker.id = 0; }
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
//     if (id == 0)
//       marker.color.r = 1.0;
//     if (id == 2)
//       marker.color.g = 1.0;
    if (id == -2) { marker.color.b = 1.0; }
//     if (id == 1)
//       marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
//     marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub.publish( marker );
  }


  void pub_circle_with_radius(double x, double y, double radius)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = transform_link;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z_coordinate / 2;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = z_coordinate;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    vis_pub.publish( marker );
  }

  void pub_circle(double x, double y, double radius, bool isLeft, int id)
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
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;
    marker.scale.x = radius;
    marker.scale.y = radius;
//     marker.scale.z = 0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    if (isLeft)
    {
      marker.color.r = 1.0;
      if (id == 2) { marker.color.b = 1.0; }
//       marker.color.g = 0.0;
    }
    else
    {
//       marker.color.r = 0.0;
      marker.color.g = 1.0;
      if (id == 3) { marker.color.b = 1.0; }
    }
//     marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
//     marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub.publish( marker );

  }

//   void checkPeopleId()
//   {
//     for (Leg& l : legs) {
//       if (!l.hasPair()) { continue; }
//
//     }
//   }

  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    pub_border_square();
//     pub_line((x_upper_limit - x_lower_limit) / 2, y_lower_limit);
//     pub_line((x_upper_limit - x_lower_limit) / 2, y_upper_limit);
//     pub_line(x_lower_limit, (y_upper_limit - y_lower_limit) / 2);
//     pub_line(x_upper_limit, (y_upper_limit - y_lower_limit) / 2);
    sensor_msgs::PointCloud2 cloudFromScan, tfTransformedCloud;

    if (!laserScanToPointCloud2(scan, cloudFromScan)) { predictLegs(); return; }

    if (!tfTransformOfPointCloud2(scan, cloudFromScan, tfTransformedCloud)) { predictLegs(); return; }

    sensor_msgs_point_cloud_publisher.publish(tfTransformedCloud);

    pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(tfTransformedCloud, *pcl_pc2);

    PointCloud cloudXYZ, filteredCloudXYZ;
    pcl::fromPCLPointCloud2(*pcl_pc2, cloudXYZ);

    if (!filterPCLPointCloud(cloudXYZ, filteredCloudXYZ)) { predictLegs(); return; }

    PointCloud cluster_centroids;

    // clustering
    if (!clustering(filteredCloudXYZ, cluster_centroids)) { predictLegs(); return; }


    ROS_WARN("vor: ");
    printLegsInfo();
    if (isOnePersonToTrack) {
      matchLegCandidates(cluster_centroids);
    } else {
      gnn_munkres(cluster_centroids);
    }
    ROS_WARN("nach: ");
    printLegsInfo();
    visLegs();
//     checkPeopleId();
    PointCloud new_people;
    std::vector<int> new_people_idx;
    findPeople(new_people, new_people_idx);

//     gnn_munkres_people(new_people, new_people_idx);

   vis_people();
//     vis_persons();


//     GNN(cluster_centroids);
//     pcl_cloud_publisher.publish(filteredCloudXYZ.makeShared());

//     PointCloud::Ptr left(new PointCloud());
//     left->header = filteredCloudXYZ.header;
//
//     PointCloud::Ptr right(new PointCloud());
//     right->header = filteredCloudXYZ.header;
//
//     sortPointCloudToLeftAndRight(filteredCloudXYZ, left, right);
//
// //     pubCircularityAndLinearity(left, true);
// //     pubCircularityAndLinearity(right, false);
//
//     if (!useKalmanFilterAndPubCircles(left, true)) { return; }
//     if (!useKalmanFilterAndPubCircles(right, false)) { return; }


  }
};


//   filters::MultiChannelFilterBase<double>* f;

int main(int argc, char **argv)
{
  ros::init(argc, argv,"leg_tracker");
  ros::NodeHandle nh("~");
  LegDetector ld(nh);
  ros::spin();

//     f = new iirob_filters::MultiChannelKalmanFilter<double>();
//     f->configure();
//
//     for (int i = 0; i < 100; ++i)
//     {
//
//     std::vector<double> v, out;
//     v.push_back(i);
//     v.push_back(i + 1);
//
//     ROS_INFO("IN: %f, %f", v[0], v[1]);
//
//
//     f->update(v, out);
//
//     ROS_INFO("OUT: %f, %f", out[0], out[1]);
//
//
//     }
  return 0;
}
