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



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


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
    
    sub = nh_.subscribe<sensor_msgs::LaserScan>(scan_topic, 10, &LegDetector::processLaserScan, this);
    pub = nh_.advertise<sensor_msgs::PointCloud2> ("scan2cloud", 10);
    pub2 = nh_.advertise<PointCloud> ("scan2pclCloud", 100);
    vis_pub = nh_.advertise<visualization_msgs::Marker>("leg_circles", 0);

  }
  ~LegDetector() {}
  
  void handleNotSetParameter(std::string parameter)
  {
      ROS_ERROR("Parameter %s not set, shutting down node...", parameter.c_str());
      nh_.shutdown();
  }
  
  
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
//     if(!tfListener.waitForTransform(scan->header.frame_id, "/base_link",
//           scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
//           ros::Duration(1.0))){
//        return;
//     }

    // project the laser into a point cloud
    
    sensor_msgs::PointCloud2 cloud;
    
//     cloud.header = scan->header;

    // project the scan into a point cloud
//     try
//     {
//       projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, cloud, *tf_);
//     }
//     catch (tf::TransformException &ex)
//     {
//       ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
// 	      ex.what());
    
  
    
    projector_.projectLaser(*scan, cloud);
    
//   }
//     projector_.transformLaserScanToPointCloud("/base_link", *scan, cloud, tfListener);
    
 
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(transform_link, scan->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    sensor_msgs::PointCloud2 cloud_transformed;
    
    tf2::doTransform(cloud, cloud_transformed, transformStamped);


//     pcl_ros::transformPointCloud(pcl_pc_laser, pcl_transformed, transform);
  
//     pcl_acc+=pcl_transformed; 
    
    
    
    pub.publish(cloud_transformed); 
    
//     const sensor_msgs::PointCloud2ConstPtr& kinect_output
    pcl::PCLPointCloud2::Ptr pc2 (new pcl::PCLPointCloud2());

    pcl_conversions::toPCL (cloud_transformed, *pc2);
/*
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (pc2);
    sor.setLeafSize(0.01f,0.01f,0.01f);
    sor.filter(*cloud_filtered);// filter the point cloud*/

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);//object cloud for point cloud type XYZ
    PointCloud cloudXYZ;
    pcl::fromPCLPointCloud2(*pc2, cloudXYZ);

    
    PointCloud cloud_filtered;
    PointCloud input_cloud;
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloudXYZ.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_lower_limit, x_upper_limit);
    //pass.setFilterLimitsNegative (true);
    pass.filter (cloud_filtered);
    pass.setInputCloud(cloud_filtered.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_lower_limit, y_upper_limit);
    pass.filter (input_cloud);
    
//     pub2.publish(input_cloud.makeShared());
    
    PointCloud::Ptr left(new PointCloud());
    left->header = input_cloud.header;
    PointCloud::Ptr right(new PointCloud());
    right->header = input_cloud.header;
    
    const int cluster_size = 2;
    
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
    
    int leftId = 0;
    
    cv::Vec3f cv_center1 = centers.at<cv::Vec3f>(0);
    cv::Vec3f cv_center2 = centers.at<cv::Vec3f>(1);
    
    // compare two centers 
    // cv_center1[0],cv_center1[1] && cv_center2[0],cv_center2[1]

    // for example
    leftId = cv_center1[1] > cv_center2[1];

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
    
    
    //pub2.publish(right);
    
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model_c_left(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (left));
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model_c_right(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (right));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_left (model_c_left);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_right (model_c_right);

    ransac_left.setDistanceThreshold (.01);
    ransac_right.setDistanceThreshold (.01);

    try
    {
      ransac_left.computeModel();
      ransac_right.computeModel();
    }
    catch (...)
    {
      ROS_ERROR("Ransac: Compute model has failed!");
      return;
    }
	    
    // get the radius of the circles
//     pcl::ModelCoefficients circle_coeff;
    Eigen::VectorXf circle_coeff;
    
    PointCloud toPub;
    toPub.header = input_cloud.header;

    std::vector<int> samples;
    std::vector<int> inliers;
    
    ransac_left.getInliers(inliers);
    for (int i : inliers) 
    {
      toPub.points.push_back(left->points[i]);
    }
    
    if (inliers.size() < 3)
    {
      ROS_ERROR("The number of inliers is too small!");
      return;
    }
      
    samples.push_back(inliers[0]);
    samples.push_back(inliers[1]);
    samples.push_back(inliers[2]);

    // samples must have 3 indizes
//     ransac_left.computeModelCoefficients (samples, &circle_coeff);
    ransac_left.getModelCoefficients (circle_coeff);
//     vtkSmartPointer<vtkDataSet> data = pcl::visualization::create2DCircle (circle_coeff);

    float center_x_left = circle_coeff(0);
    float center_y_left = circle_coeff(1);
    pub_circle(center_x_left, center_y_left, circle_coeff(2));
    
    pcl::PointXYZ point;
    point.x = center_x_left;
    point.y = center_y_left;
    point.z = 0.0;
    toPub.points.push_back(point);
    
    
    ransac_right.getInliers(inliers);
    for (int i : inliers) 
    {
      
      toPub.points.push_back(right->points[i]);
    }
    
    if (inliers.size() < 3)
    {
      ROS_ERROR("The number of inliers is too small!");
      return;
    }
    
    samples.push_back(inliers[0]);
    samples.push_back(inliers[1]);
    samples.push_back(inliers[2]);

    // samples must have 3 indizes
//     ransac_right.computeModelCoefficients (samples, &circle_coeff);
    ransac_right.getModelCoefficients (circle_coeff);
//     vtkSmartPointer<vtkDataSet> data = pcl::visualization::create2DCircle (circle_coeff);

    float center_x_right = circle_coeff(0);
    float center_y_right = circle_coeff(1);
    
    pub_circle(center_x_right, center_y_right, circle_coeff(2));
    
    point.x = center_x_right;
    point.y = center_y_right;
    point.z = 0.0;
    toPub.points.push_back(point);
    
    pub2.publish(toPub.makeShared());
    
    
  }
  
  void pub_circle(float x, float y, float radius)
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
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub.publish( marker );

  }
  
  

// /* 
//  *  update Kalman filter model
//  */
// ...
// 
// 
// 
// 
// 
// 
// pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat OpencVPointCloud)
//  {
// 	 /*
// 	 *  Function: Get from a Mat to pcl pointcloud datatype
// 	 *  In: cv::Mat
// 	 *  Out: pcl::PointCloud
// 	 */
// 
// 	 //char pr=100, pg=100, pb=100;
// 	 pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::pointcloud<pcl::pointXYZ>);
// 
// 	 for(int i=0;i<OpencVPointCloud.cols;i++)
// 	 {
// 		//std::cout<<i<<endl;
// 
// 		pcl::PointXYZ point;
// 		point.x = OpencVPointCloud.at<float>(0,i);
// 		point.y = OpencVPointCloud.at<float>(1,i);
// 		point.z = OpencVPointCloud.at<float>(2,i);
// 
// 		// when color needs to be added:
// 		//uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
// 		//point.rgb = *reinterpret_cast<float*>(&rgb);
// 
// 		point_cloud_ptr -> points.push_back(point);
// 
// 
// 	 }
// 	 point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
// 	 point_cloud_ptr->height = 1;
// 
// 	 return point_cloud_ptr;
// 
//  }
//   
//   
//   
//   
  
  
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