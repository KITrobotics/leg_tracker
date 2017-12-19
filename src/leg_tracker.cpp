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


/*
 * TODO:
 * N1 ~ new people appeared
 * N2 ~ continue tracking of already tracked people which were occluded
 * N1 = 0.7 and N2 = 1.2
 * 
 */




typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//typedef std::map<int, std::pair<pcl::PointXYZ, pcl::PointXYZ> > PeopleMap;

const int cluster_size = 2;

class LegDetector
{
private:
  ros::Subscriber sub;
  laser_geometry::LaserProjection projector_;
  ros::Publisher sensor_msgs_point_cloud_publisher;
  ros::Publisher pcl_cloud_publisher;
  ros::Publisher pos_vel_acc_lleg_pub;
  ros::Publisher pos_vel_acc_rleg_pub;
  ros::Publisher marker_array_publisher;
  
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
  pcl::PointXYZ person_center;
  cv::Point2f left_leg_prediction;
  cv::Point2f right_leg_prediction;
  int legs_gathered;
  int min_observations;
  int min_predictions;
  double radius_of_person;
  int id_counter;
  double z_coordinate;
  
  
  
  //PeopleMap persons;
  std::vector<Leg> removed_legs;
  std::vector<Leg> temporary_legs;
  std::vector<std::pair<Leg, Leg> > persons;
  
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
    nh_.param("scan_topic", scan_topic, std::string("/base_laser_rear/scan"));
    nh_.param("transform_link", transform_link, std::string("base_link"));
    nh_.param("x_lower_limit", x_lower_limit, 0.0);
    nh_.param("x_upper_limit", x_upper_limit, 0.5);
    nh_.param("y_lower_limit", y_lower_limit, -0.5);
    nh_.param("y_upper_limit", y_upper_limit, 0.5);
    nh_.param("ransac_dist_threshold", ransac_dist_threshold, 0.1);
    nh_.param("circle_fitting", circle_fitting, std::string("centroid"));
    nh_.param("leg_radius", leg_radius, 0.1);
    nh_.param("min_observations", min_observations, 4);
    nh_.param("min_predictions", min_predictions, 7);
    nh_.param("radius_of_person", radius_of_person, 1.0);
    nh_.param("z_coordinate", z_coordinate, 0.178);
    
    legs_gathered = id_counter = 0;
    
    sub = nh_.subscribe<sensor_msgs::LaserScan>(scan_topic, 1, &LegDetector::processLaserScan, this);
    sensor_msgs_point_cloud_publisher = nh_.advertise<sensor_msgs::PointCloud2> ("scan2cloud", 10);
    pcl_cloud_publisher = nh_.advertise<PointCloud> ("scan2pclCloud", 100);
    vis_pub = nh_.advertise<visualization_msgs::Marker>("leg_circles", 10);
    pos_vel_acc_lleg_pub = nh_.advertise<std_msgs::Float64MultiArray>("pos_vel_acc_lleg", 10);
    pos_vel_acc_rleg_pub = nh_.advertise<std_msgs::Float64MultiArray>("pos_vel_acc_rleg", 10);
    marker_array_publisher = nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 10);
//       
  }
  
  void handleNotSetParameter(std::string parameter)
  {
      ROS_ERROR("Parameter %s not set, shutting down node...", parameter.c_str());
      nh_.shutdown();
  }
  
  
  bool laserScanToPointCloud2(const sensor_msgs::LaserScan::ConstPtr& scan, sensor_msgs::PointCloud2& cloud)
  {
    if (!scan) { ROS_INFO("Laser scan pointer was not set!"); return false; }
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
      ROS_INFO("Filtering: Too small number of points in the input PointCloud!"); 
      return false; 
    }
    
//     PointCloud::Ptr path_throw_filtered_x (new PointCloud());
//     PointCloud::Ptr path_throw_filtered_y (new PointCloud());
//     PointCloud::Ptr sor_filtered (new PointCloud());
//       
    PointCloud path_throw_filtered_x;
    PointCloud path_throw_filtered_y;
//     PointCloud sor_filtered;
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in.makeShared());
    pass.setFilterFieldName("x");
    ROS_WARN("x_lower_limit: %f, x_upper_limit: %f", x_lower_limit, x_upper_limit);
    pass.setFilterLimits(x_lower_limit, x_upper_limit);
    pass.setFilterLimitsNegative (false);
    pass.filter (path_throw_filtered_x);
    pass.setInputCloud(path_throw_filtered_x.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_lower_limit, y_upper_limit);
    pass.filter (path_throw_filtered_y);
    
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(path_throw_filtered_y.makeShared());
    outrem.setRadiusSearch(0.02);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    outrem.filter (out);
    
    if (out.points.size() < 5) 
    { 
      ROS_INFO("Filtering: Too small number of points in the resulting PointCloud!"); 
      return false; 
    }
    return true;
  }
  
  void initLeg(const pcl::PointXYZ& p)
  {
    Leg l(min_predictions, min_observations);
    if (!l.configure(p)) { ROS_ERROR("Configuring failed!"); return; }
    legs.push_back(l);
    ROS_INFO("AFTER initLeg");
    printLegsInfo();
  }  
  
  void updateLeg(Leg& l, const pcl::PointXYZ& p, std::vector<double>& in, std::vector<double>& out)
  {
    ROS_INFO("updateLeg");
    printLegsInfo();
    in.clear();
    in.push_back(p.x); in.push_back(p.y);
    l.update(in, out);
    ROS_INFO("AFTER updateLeg");
    printLegsInfo();
  }
  
  
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
  }
  
  
  void printLegsInfo()
  {
	int people = 0;
    for (Leg& l : legs)
    {
      ROS_INFO("peopleId: %d, pos: (%f, %f, %f), predictions: %d, observations: %d, hasPair: %d", 
      l.getPeopleId(), l.getPos().x, l.getPos().y, l.getPos().z, l.getPredictions(), l.getObservations(), l.hasPair());
    }
  }
  
  void visLegs(PointCloud& cloud)
  {
    cloud.points.clear();
    visualization_msgs::MarkerArray ma;
    int id = 0;
    for (Leg& l : legs)
    {
      ROS_INFO("VISlegs peopleId: %d, pos: (%f, %f, %f), predictions: %d, observations: %d, hasPair: %d", 
      l.getPeopleId(), l.getPos().x, l.getPos().y, l.getPos().z, l.getPredictions(), l.getObservations(), l.hasPair());
      cloud.points.push_back(l.getPos());
      ma.markers.push_back(getMarker(l.getPos().x, l.getPos().y, id));
      id++;
    }
    marker_array_publisher.publish(ma);
    pcl_cloud_publisher.publish(cloud.makeShared());
  }
  
  
  visualization_msgs::Marker getMarker(double x, double y, int id)
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
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;
    marker.scale.x = leg_radius;
    marker.scale.y = leg_radius;
    marker.scale.z = z_coordinate;
    marker.color.a = 1.0; // Don't forget to set the alpha!
//     if (id == 0)
//       marker.color.r = 1.0; 
//     if (id == 2)
//       marker.color.g = 1.0; 
//     if (id == 1)
//       marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
//     marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    return marker;
  }
  
  void printClusterInfo(const PointCloud& cluster_centroids)
  {
    ROS_INFO("Clusters:");
    for (int i = 0; i < cluster_centroids.points.size(); i++)
    {
      ROS_INFO("cluster %d: (%f, %f, %f)", i, cluster_centroids.points[i].x, cluster_centroids.points[i].y, cluster_centroids.points[i].z);
    }
  }
  
  void matchLegCandidates(PointCloud cluster_centroids)
  {
    printClusterInfo(cluster_centroids);
    PointCloud predictions;
    predictions.header = cluster_centroids.header;
    computeKalmanFilterPredictions(predictions);
    ROS_INFO("predictions: %d, cluster_centroids: %d", (int) predictions.points.size(), (int) cluster_centroids.points.size());
    if (predictions.points.size() == 0 && cluster_centroids.points.size() == 0) { return; }
    
    if (cluster_centroids.points.size() == 0)
    {
      for (int i = 0; i < legs.size(); i++)
      {
		predictLeg(i);
      }
      return;
    }
    
    
    std::vector<double> in, out;
    
    if (predictions.points.size() == 0)
    { 
    
      for (pcl::PointXYZ p : cluster_centroids.points)
      {
		initLeg(p);
      }
      return;
    }
    
    for (int i = 0; i < legs.size(); i++) 
    {
      if (cluster_centroids.points.size() == 0) { predictLeg(i); continue; }
      pcl::PointXYZ prediction, nearest;
      prediction = legs[i].computePrediction();
      double radius = (2 + legs[i].getPredictions()) * leg_radius;
      if (!findAndEraseMatch(prediction, cluster_centroids, nearest, radius)) { predictLeg(i); }
      else { updateLeg(legs[i], nearest, in, out); }
    }
	
    findPeople();
    
    for (pcl::PointXYZ p : cluster_centroids.points)
    {
      initLeg(p);
    }
    
    
  }
  
  void findPeople()
  {
    ROS_INFO("findPeople");
    for (int i = 0; i < legs.size(); i++) 
    {
      if (legs[i].getObservations() >= min_observations && !legs[i].hasPair())
      {
	      findSecondLeg(i);
      }
    }
  }
  
  void findSecondLeg(int fst_leg)
  {
    ROS_INFO("findSecondLeg");
	PointCloud potential_legs;
	std::vector<int> indices;
	for (int i = 0; i < legs.size(); i++) 
	{
		if (i == fst_leg || legs[i].hasPair())
		{
			continue;
		}
		
		potential_legs.points.push_back(legs[i].getPos());
		indices.push_back(i);
	}
	
	if (potential_legs.size() == 0) { return; }
	
	int snd_leg = findMatch(legs[fst_leg].getPos(), potential_legs, indices);
	if (snd_leg == -1) { ROS_INFO("Could not find second leg!"); return; }
	
	setPeopleId(fst_leg, snd_leg);
	
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
  
  void setPeopleId(int fst_leg, int snd_leg)
  {
	  
	  int id = -1;
	  bool isIdSet = false;
	  isIdSet = legs[fst_leg].getPeopleId() != -1 || legs[snd_leg].getPeopleId() != -1;
	  ROS_WARN("ids: %d, %d", legs[fst_leg].getPeopleId(), legs[snd_leg].getPeopleId());
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
		  // legs from not same people
		  //return;
			id = legs[fst_leg].getPeopleId() < legs[snd_leg].getPeopleId() ? 
				legs[fst_leg].getPeopleId() : legs[snd_leg].getPeopleId();
				
			eraseRemovedLeg(id);
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
	  //std::pair<PeopleMap::iterator, bool> ret;
	  //std::pair <pcl::PointXYZ, pcl::PointXYZ> leg_points = std::make_pair(legs[fst_leg].getPos(), legs[fst_leg].getPos());
	  //ret = persons.insert ( std::pair<int, std::pair <pcl::PointXYZ, pcl::PointXYZ> >(id, leg_points) );
	  //if (ret.second==false) {
		//std::cout << "element 'z' already existed";
		//std::cout << " with a value of " << ret.first->second << '\n';
	  //}
	  //persons.add(id, std::make_pair(legs[fst_leg].getPos(), legs[fst_leg].getPos()));
  }
  
  void printCloudPoints(const PointCloud& cloud)
  {
    for (int i = 0; i < cloud.points.size(); i++)
    {
      ROS_INFO("Point %d: (%f, %f, %f)", i, cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    }
  }
  
  
  double calculateAndPubOval(double x1, double y1, double x2, double y2, int id)
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
	    pub_oval(x, y, scale_x, scale_y, orientation_x, orientation_y, orientation_z, orientation_w, id);
  }
 
  
  void vis_people()
  {
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
			  calculateAndPubOval(legs[i].getPos().x, legs[i].getPos().y, l.getPos().x, l.getPos().y, id);
					      
			  ROS_INFO("VISpeople peopleId: %d, pos1: (%f, %f, %f), pos2removed: (%f, %f, %f), predictions: (%d, %d), observations: (%d, %d), hasPair: (%d, %d)", 
			  id, legs[i].getPos().x, legs[i].getPos().y, legs[i].getPos().z, l.getPos().x, l.getPos().y, l.getPos().z, legs[i].getPredictions(), l.getPredictions(), 
				   legs[i].getObservations(), l.getObservations(), legs[i].hasPair(), l.hasPair());
			}
		}
		
	}
	  
      if (legs[i].hasPair())
      {
	for (int j = i + 1; j < legs.size(); j++)
	{
	  if (legs[j].getPeopleId() == id)
	  {
	    calculateAndPubOval(legs[i].getPos().x, legs[i].getPos().y, legs[j].getPos().x, legs[j].getPos().y, id);
		  
		  
	    ROS_INFO("VISpeople peopleId: %d, pos1: (%f, %f, %f), pos2: (%f, %f, %f), predictions: (%d, %d), observations: (%d, %d), hasPair: (%d, %d)", 
	    id, legs[i].getPos().x, legs[i].getPos().y, legs[i].getPos().z, 
		     legs[j].getPos().x, legs[j].getPos().y, legs[j].getPos().z, 
		     legs[i].getPredictions(), legs[j].getPredictions(), 
		      legs[i].getObservations(), legs[j].getObservations(), 
		     legs[i].hasPair(), legs[j].hasPair());
	  }
	}
      }
    }
  }
  
  int findMatch(const pcl::PointXYZ& searchPoint, const PointCloud& cloud, const std::vector<int>& indices)
  {
    ROS_INFO("findMatch for (%f, %f, %f) with radius %f", searchPoint.x, searchPoint.y, searchPoint.z, radius_of_person);
    printCloudPoints(cloud);
    int out_index = -1;
    if (cloud.points.size() == 0) { return out_index; }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud.makeShared());
    std::vector<int> pointIdxRadius;
    std::vector<float> pointsSquaredDistRadius;
    int count = kdtree.radiusSearch (searchPoint, radius_of_person, pointIdxRadius, pointsSquaredDistRadius);
	
    if (count == 0) { return -1; }
    
    if (count > 1) 
    { 
      ROS_INFO("mahalanobis");
      // interesting more than one leg matched
      // use mahalanobis distance
    }
    
    int K = 1;
    std::vector<int> pointsIdx(K);
    std::vector<float> pointsSquaredDist(K);
    kdtree.nearestKSearch (searchPoint, K, pointsIdx, pointsSquaredDist);
    ROS_INFO("legs.size: %d, pointsIdx[0]: %d, indices[pointsIdx[0]]: %d, pointsSquaredDist: %f", 
      (int) legs.size(), pointsIdx[0], indices[pointsIdx[0]], pointsSquaredDist[0]);
    if (pointsSquaredDist[0] <= radius_of_person && pointsSquaredDist[0] >= 0.5 * leg_radius) {
      out_index = indices[pointsIdx[0]];
    }
    return out_index;  
  }
  
  bool findAndEraseMatch(const pcl::PointXYZ& searchPoint, PointCloud& cloud, pcl::PointXYZ& out, double radius)
  {
    ROS_INFO("findAndEraseMatch");
    if (cloud.points.size() == 0) { return false; }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud.makeShared());
    std::vector<int> pointIdxRadius;
    std::vector<float> pointsSquaredDistRadius;
    // radius search
    int count = kdtree.radiusSearch (searchPoint, radius, pointIdxRadius, pointsSquaredDistRadius);
    if (count == 0) { return false; }
    if (count > 1) 
    { 
      //interesting more than one point matched
    }
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
    for (Leg& l : legs) 
    {
      predictions.points.push_back(l.computePrediction());
    }
  }
  
  bool clustering(const PointCloud& cloud, PointCloud& cluster_centroids)
  {
    
    if (cloud.points.size() < 4) { ROS_INFO("Clustering: Too small number of points!"); return false; }
//     pcl::PCDWriter writer;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud.makeShared());
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.04); // 4cm
    ec.setMinClusterSize (4);
    ec.setMaxClusterSize (100);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud.makeShared());
    ec.extract (cluster_indices);
    
    cluster_centroids.header = cloud.header;
    cluster_centroids.points.clear();
    
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
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
	pcl::PointXYZ p(c[0], c[1], 0.0);
	cluster_centroids.points.push_back(p);
// 	pub_circle_with_id(c[0], c[1], j);
      }
//       std::stringstream ss;
//       ss << "cloud_cluster_" << j << ".pcd";
//       writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      j++;
    }
//     pcl_cloud_publisher.publish(cluster_centroids);

 //   std::cout << "clusters: " << j << " datas." << std::endl;	
    return true;
  }
  
  void sortPointCloudToLeftAndRight(const PointCloud& input_cloud, PointCloud::Ptr left, PointCloud::Ptr right)
  {
    
//     #if CV_MAJOR_VERSION == 2
    // do opencv 2 code

    
    // convert input cloud to opencv Mat
    cv::Mat points(input_cloud.size(),1, CV_32FC3);
    std::size_t idx = 0;

//     std::cout << "Input model points:\n";
    for(PointCloud::const_iterator it = input_cloud.begin(); it != input_cloud.end(); ++it, ++idx)
    {
	    points.at<cv::Vec3f>(idx,0) = cv::Vec3f(it->x,it->y,it->z);
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
  }
  
  std::vector<double> computeCentroids(PointCloud::Ptr cloud)
  {
    std::vector<double> result;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud, centroid);
    result.push_back(centroid(0));
    result.push_back(centroid(1));
    return result;
  }
  
  std::vector<double> computeRansacPubInliersAndGetCenters(PointCloud::Ptr cloud)
  {
    std::vector<double> centers;
    PointCloud toPub;
    toPub.header = cloud->header;
    
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model(
      new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);

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
    
    
    pcl::PointXYZ point;
    point.x = circle_coeff(0);
    point.y = circle_coeff(1);
    point.z = 0.0;
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
  
  bool useKalmanFilterAndPubCircles(const PointCloud::Ptr cloud, bool isLeft)
  {
    const int num_elements_measurement = 2;
    const int num_elements_kalman = 6;
    std::vector<double> centerOfLegMeasurement;
    std::vector<double> centerOfLegKalman;
//     if (circle_fitting == "ransac")
//     {
//       centerOfLegMeasurement = computeRansacPubInliersAndGetCenters(cloud);
//     }
//     else
//     {
//       centerOfLegMeasurement = computeCentroids(cloud);
//     }
    if (!computeCircularity(cloud, centerOfLegMeasurement)) { return false; }
    
    if (centerOfLegMeasurement.size() < num_elements_measurement) 
    { 
      ROS_ERROR("Center (measurement) does not have enough elements!"); 
      return false; 
    }
    
//     if (abs(centerOfLegMeasurement[0] - centerOfLegLastMeasurement[0]) > ...)
//     {
//       
//     }
    std::vector<double> prediction;
    if (isLeft) { left_leg_filter->predict(prediction); left_leg_prediction = cv::Point2f(prediction[0], prediction[1]); }
    else { right_leg_filter->predict(prediction); right_leg_prediction = cv::Point2f(prediction[0], prediction[1]); }
    pub_circle(prediction[0], prediction[1], leg_radius, isLeft, isLeft ? 2 : 3);
    
    if (isLeft) { left_leg_filter->update(centerOfLegMeasurement, centerOfLegKalman); }
    else { right_leg_filter->update(centerOfLegMeasurement, centerOfLegKalman); }
    
//     pub_circle(centerOfLegMeasurement[0], centerOfLegMeasurement[1], leg_radius, isLeft, isLeft ? 0 : 1);

    
    
    if (centerOfLegKalman.size() < num_elements_kalman) 
    { 
      ROS_ERROR("Centers (kalman) do not have enough elements!"); 
      return false; 
    }
    
//     pub_circle(centerOfLegKalman[0], centerOfLegKalman[1], leg_radius, isLeft, isLeft ? 2 : 3);
    if (legs_gathered == 1) {
      person_center.x = (person_center.x + centerOfLegKalman[0]) / 2;
      person_center.y = (person_center.y + centerOfLegKalman[1]) / 2;
      legs_gathered++;
    } else {
      if (legs_gathered == 2) {
	pub_circle(person_center.x, person_center.y, 0.7, isLeft, 0);
      } 
      person_center.x = centerOfLegKalman[0];
      person_center.y = centerOfLegKalman[1];
      legs_gathered = 1;
    }  
    
    pub_leg_posvelacc(centerOfLegKalman, isLeft);
    
    return true;
  }
  
  bool computeCircularity(const PointCloud::Ptr cloud, std::vector<double>& center)
  {
    int min_points = 4;
    int num_points = cloud->points.size();
    if (num_points < min_points) { ROS_ERROR("Circularity and Linerity: Too small number of points!"); return false; }
    double x_mean, y_mean;
    CvMat* A = cvCreateMat(num_points, 3, CV_64FC1);
    CvMat* B = cvCreateMat(num_points, 1, CV_64FC1);
    
//     CvMat* points = cvCreateMat(num_points, 2, CV_64FC1);
    
    int j = 0;
    for (pcl::PointXYZ p : cloud->points)
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
  
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    sensor_msgs::PointCloud2 cloudFromScan, tfTransformedCloud;
    
    if (!laserScanToPointCloud2(scan, cloudFromScan)) { return; }
   
    if (!tfTransformOfPointCloud2(scan, cloudFromScan, tfTransformedCloud)) { return; }
    
    sensor_msgs_point_cloud_publisher.publish(tfTransformedCloud); 
    
    pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(tfTransformedCloud, *pcl_pc2);

    PointCloud cloudXYZ, filteredCloudXYZ;
    pcl::fromPCLPointCloud2(*pcl_pc2, cloudXYZ);

    if (!this->filterPCLPointCloud(cloudXYZ, filteredCloudXYZ)) { return; }
    
    PointCloud cluster_centroids;
    if (!clustering(filteredCloudXYZ, cluster_centroids)) { return; }
    matchLegCandidates(cluster_centroids);
    visLegs(cluster_centroids);
    vis_people();
    
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
  
  void pub_oval(double x, double y, double scale_x, double scale_y, double orientation_x, 
    double orientation_y, double orientation_z, double orientation_w, int id)
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
  
protected:
    void ROS_ERROR ( const char* arg1 );
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