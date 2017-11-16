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
#include <pcl/filters/radius_outlier_removal.h>
#include <iostream>
#include "kalman_filter.h"
#include <pcl/common/centroid.h>





/*
 * TODO:
 * N1 ~ new people appeared
 * N2 ~ continue tracking of already tracked people which were occluded
 * N1 = 0.7 and N2 = 1.2
 * 
 */




typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const int cluster_size = 2;

class LegDetector
{
private:
  ros::Subscriber sub;
  laser_geometry::LaserProjection projector_;
  ros::Publisher pub;
  ros::Publisher pub2;
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

  // Construct the filter
  filters::MultiChannelFilterBase<double>* left_leg_filter;
  filters::MultiChannelFilterBase<double>* right_leg_filter;

public:
  ros::NodeHandle nh_;
  LegDetector(ros::NodeHandle nh) : nh_(nh), tfListener(tfBuffer)
  {
    nh_.param("scan_topic", scan_topic, std::string("/base_laser_rear/scan"));
    nh_.param("transform_link", transform_link, std::string("base_link"));
    nh_.param("x_lower_limit", x_lower_limit, 0.0);
    nh_.param("x_upper_limit", x_upper_limit, 0.5);
    nh_.param("y_lower_limit", y_lower_limit, -0.5);
    nh_.param("y_upper_limit", y_upper_limit, 0.5);
    nh_.param("ransac_dist_threshold", ransac_dist_threshold, 0.1);
    nh_.param("circle_fitting", circle_fitting, std::string("centroid"));
    
    sub = nh_.subscribe<sensor_msgs::LaserScan>(scan_topic, 10, &LegDetector::processLaserScan, this);
    pub = nh_.advertise<sensor_msgs::PointCloud2> ("scan2cloud", 10);
    pub2 = nh_.advertise<PointCloud> ("scan2pclCloud", 100);
    vis_pub = nh_.advertise<visualization_msgs::Marker>("leg_circles", 0);
    
      

    int n = 6; // Number of states
    int m = 2; // Number of measurements

    double dt = 1.0/20; // Time step
    
    Eigen::MatrixXd A(n, n); // System dynamics matrix
//     Eigen::MatrixXd B(n, m);
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance
    
    A << 1, 0, dt, 0, pow(dt, 2) / 2, 0, 
	 0, 1, 0, dt, 0, pow(dt, 2)/2, 
	 0, 0, 1, 0, dt, 0,
	 0, 0, 0, 1, 0, dt,
	 0, 0, 0, 0, 1, 0,
	 0, 0, 0, 0, 0, 1;
//     B << (dt^2) / 2, 0, 0, (dt^2) / 2, dt, 0, 0, dt;
    C << 1, 0, 0, 0, 0, 0, 
	 0, 1, 0, 0, 0, 0;

    // Reasonable covariance matrices
    Q << 0.5, 0, 0, 0, 0, 0, 
	 0, 0.5, 0, 0, 0, 0, 
	 0, 0, 0.5, 0, 0, 0,
	 0, 0, 0, 0.5, 0, 0,
	 0, 0, 0, 0, 0.5, 0,
	 0, 0, 0, 0, 0, 0.5;
//     Q << .05, .05, .0, 
// 	 .05, .05, .0, 
// 	 .0, .0, .0;
    R << pow(0.04, 2), 0, 
	 0, pow(0.04, 2);
//     P.setIdentity();
    P << 10, 0, 0, 0, 0, 0, 
	 0, 10, 0, 0, 0, 0, 
	 0, 0, 10, 0, 0, 0,
	 0, 0, 0, 10, 0, 0,
	 0, 0, 0, 0, 10, 0,
	 0, 0, 0, 0, 0, 10;
//     P << .1, .1, .1, 
// 	 .1, 10000, 10, 
// 	 .1, 10, 100;
    
    Eigen::VectorXd x0(n);
    x0 << 0, 0, 0;
    
    left_leg_filter = new filters::KalmanFilter<double>(0.0, A, C, Q, R, P, x0);
    left_leg_filter->configure();
    right_leg_filter = new filters::KalmanFilter<double>(0.0, A, C, Q, R, P, x0);
    right_leg_filter->configure();
  }
  ~LegDetector() {}
  
  void handleNotSetParameter(std::string parameter)
  {
      ROS_ERROR("Parameter %s not set, shutting down node...", parameter.c_str());
      nh_.shutdown();
  }
  
  
  void laserScanToPointCloud2(const sensor_msgs::LaserScan::ConstPtr& scan, sensor_msgs::PointCloud2& cloud)
  {
    projector_.projectLaser(*scan, cloud);
  }
  
  void tfTransformOfPointCloud2(const sensor_msgs::LaserScan::ConstPtr& scan, 
				sensor_msgs::PointCloud2& from, sensor_msgs::PointCloud2& to)
  {
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(transform_link, scan->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    tf2::doTransform(from, to, transformStamped);
  }
  
  void filterPCLPointCloud(PointCloud& in, PointCloud& out)
  {
    PointCloud path_throw_filtered_x;
    PointCloud path_throw_filtered_y;
    PointCloud sor_filtered;
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_lower_limit, x_upper_limit);
    //pass.setFilterLimitsNegative (true);
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
    
    if (out.points.size() == 0) 
    { 
      ROS_INFO("Point cloud has 0 points!");
      return;
    }
  }
  
  void sortPointCloudToLeftAndRight(PointCloud& input_cloud, PointCloud::Ptr left, PointCloud::Ptr right)
  {
    
    #if CV_MAJOR_VERSION == 2
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
    // bool success = false;

    int max_cluster_size = input_cloud.size() > cluster_size ? cluster_size : input_cloud.size(); 
    
    // use opencv kmeans to extract the cluster center, since pcl 1.7.2 does not have kmeans
    cv::kmeans(points, max_cluster_size, labels, 
	    cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10e4, 1e-4),
	    attempts, cv::KMEANS_RANDOM_CENTERS, centers);

    // std::cerr << "Kmeans size: " << centers.size() << std::endl;
    // success = centers.rows > 0;
    
    if (centers.rows != 2)
    {
      ROS_ERROR("KMeans: The number of rows is not valid!");
      return;
    }
    

    
    cv::Vec3f cv_center1 = centers.at<cv::Vec3f>(0);
    cv::Vec3f cv_center2 = centers.at<cv::Vec3f>(1);
    
    // compare two centers 
    // cv_center1[0],cv_center1[1] && cv_center2[0],cv_center2[1]

    // for example
    // is y of the first center bigger than y of the second center?
    int leftId = cv_center1[1] > cv_center2[1] ? 0 : 1;
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
    
    
    #elif CV_MAJOR_VERSION == 3
    // do opencv 3 code
    #endif
    
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
  
  std::vector<double> computeRansacModelsAngGetCenters(PointCloud::Ptr left, PointCloud::Ptr right)
  {
    std::vector<double> centers;
    PointCloud toPub;
    toPub.header = left->header;
    
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model_c_left(
      new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (left));
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model_c_right(
      new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (right));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_left (model_c_left);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_right (model_c_right);

    ransac_left.setDistanceThreshold (ransac_dist_threshold);
    ransac_right.setDistanceThreshold (ransac_dist_threshold);

    try
    {
      ransac_left.computeModel();
      ransac_right.computeModel();
    }
    catch (...)
    {
      ROS_ERROR("Ransac: Computing model has failed!");
      return centers;
    }
	    
    // get the radius of the circles
//     pcl::ModelCoefficients circle_coeff;
    Eigen::VectorXf circle_coeff;

    std::vector<int> samples;
    std::vector<int> inliers;
    
    ransac_left.getInliers(inliers);
    if (inliers.size() < 3)
    {
      ROS_ERROR("The number of inliers is too small!");
      return centers;
    }
    for (int i : inliers) 
    {
      toPub.points.push_back(left->points[i]);
    }
    
      
    samples.push_back(inliers[0]);
    samples.push_back(inliers[1]);
    samples.push_back(inliers[2]);

    // samples must have 3 indizes
//     ransac_left.computeModelCoefficients (samples, &circle_coeff);
    ransac_left.getModelCoefficients (circle_coeff);
//     vtkSmartPointer<vtkDataSet> data = pcl::visualization::create2DCircle (circle_coeff);
    
    centers.push_back(circle_coeff(1));
    centers.push_back(circle_coeff(2));
//     if (circle_coeff(2) > 0.1)
//     {
//       ROS_INFO("Left circle radius: %f, number of points in the left pointcloud: %d", circle_coeff(2), (int) left->points.size());
//     }
    pub_circle(circle_coeff(0), circle_coeff(1), 0.1, true, 0);
    
    
    pcl::PointXYZ point;
    point.x = circle_coeff(0);
    point.y = circle_coeff(1);
    point.z = 0.0;
    toPub.points.push_back(point);
    
    
    ransac_right.getInliers(inliers);
    if (inliers.size() < 3)
    {
      ROS_ERROR("The number of inliers is too small!");
      return centers;
    }
    for (int i : inliers) 
    {
      toPub.points.push_back(right->points[i]);
    }
    
    
    samples.push_back(inliers[0]);
    samples.push_back(inliers[1]);
    samples.push_back(inliers[2]);

    // samples must have 3 indizes
//     ransac_right.computeModelCoefficients (samples, &circle_coeff);
    ransac_right.getModelCoefficients (circle_coeff);
    
    centers.push_back(circle_coeff(1));
    centers.push_back(circle_coeff(2));
//     vtkSmartPointer<vtkDataSet> data = pcl::visualization::create2DCircle (circle_coeff);

//     int a;
//     std::cin >> a;
    pub_circle(circle_coeff(0), circle_coeff(1), 0.1, false, 1);
    
//     if (circle_coeff(2) > 0.1)
//     {
//       ROS_INFO("x: %f, y: %f, radius: %f", circle_coeff(0), circle_coeff(1), circle_coeff(2));
//       for (pcl::PointXYZ p : right->points) 
//       {
// 	ROS_INFO("Point: x = %f, y = %f", p.x, p.y);
//       }
      
//       pub2.publish(right);
//       nh_.shutdown();
//     }
    
    point.x = circle_coeff(0);
    point.y = circle_coeff(1);
    point.z = 0.0;
    toPub.points.push_back(point);
    
    pub2.publish(toPub.makeShared());
    
    return centers;
  }
  
//     std::vector<double> data_in, data_out;
//     // fill data_in
//     
//     
//     filter->update(data_in, data_out);
  
  
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    sensor_msgs::PointCloud2 cloudFromScan, tfTransformedCloud;
    
    laserScanToPointCloud2(scan, cloudFromScan);
   
    tfTransformOfPointCloud2(scan, cloudFromScan, tfTransformedCloud);
    
    pub.publish(tfTransformedCloud); 
    
    pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2());

    pcl_conversions::toPCL (tfTransformedCloud, *pcl_pc2);

    PointCloud cloudXYZ, filteredCloudXYZ;
    pcl::fromPCLPointCloud2(*pcl_pc2, cloudXYZ);

    filterPCLPointCloud(cloudXYZ, filteredCloudXYZ);
    
    pub2.publish(filteredCloudXYZ.makeShared());
    
    PointCloud::Ptr left(new PointCloud());
    left->header = filteredCloudXYZ.header;
    PointCloud::Ptr right(new PointCloud());
    right->header = filteredCloudXYZ.header;
    std::vector<double> centersOfLegsMeasurement;
    std::vector<double> centersOfLegsKalman;
    
    if (circle_fitting == "ransac")
    {
      
    }
    else
    {
      
    }
    
    sortPointCloudToLeftAndRight(filteredCloudXYZ, left, right);
    
    centersOfLegsMeasurement = computeRansacModelsAngGetCenters(left, right);
    
//     centersOfLegsMeasurement = computeCentroids(left);
    /*
    int numOfElements = 4;
    if (centersOfLegsMeasurement.size() < numOfElements) { return; }*/
    
//     std::vector<double> v1;
//     v1.push_back(centersOfLegsMeasurement[0]);
//     v1.push_back(centersOfLegsMeasurement[1]);
//     std::vector<double> v2;
//     v1.push_back(centersOfLegsMeasurement[2]);
//     v1.push_back(centersOfLegsMeasurement[3]);
//     std::vector<double> v3;
//     std::vector<double> v4;
//     
//     left_leg_filter->update(v1, v3);
//     if (centersOfLegsKalman.size() < numOfElements) { return; }
//     
//     double radius = 0.1;
//     pub_circle(centersOfLegsMeasurement[0], centersOfLegsMeasurement[1], radius, true, 0);
//     pub_circle(centersOfLegsKalman[0], centersOfLegsKalman[1], radius, true, 2);
    
//     centersOfLegsMeasurement = computeCentroids(right);
//     
//     if (centersOfLegsMeasurement.size() < numOfElements) { return; }
//     
//     right_leg_filter->update(v2, v4);
//     if (centersOfLegsKalman.size() < numOfElements) { return; }
    
//     pub_circle(centersOfLegsMeasurement[2], centersOfLegsMeasurement[3], radius, true, 1);
//     pub_circle(centersOfLegsKalman[0], centersOfLegsKalman[1], radius, true, 3);
    
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
//     marker.pose.position.z = 0;
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
  
};


int main(int argc, char **argv)
{
	ros::init(argc, argv,"leg_detection");
	ros::NodeHandle nh("~");
	LegDetector ld(nh);
	

	ROS_INFO("Execute main");
	ros::spin();

	return 0;
}