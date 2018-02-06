
#ifndef LEG_TRACKER_H
#define LEG_TRACKER_H

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
#include "Matrix.h"
#include "Munkres.h"

#endif