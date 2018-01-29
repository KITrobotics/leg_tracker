/*
 * A tracked object. Could be a person leg,
 * entire person or any arbitrary object in the laser scan.
 */
class ObjectTracked
{
public:
  unsigned int new_leg_id_num = 1;

  ObjectTracked(double x, double y, ros::Time now, double confidence,
    bool is_person, double in_free_space)
  {
    id_num = new_leg_id_num++;
    last_seen = now;
    seen_in_current_scan = true;
    times_seen = 1;
    this->confidence = confidence;
    dist_travelled = 0.;
    this->is_person = is_person;
    this->in_free_space = in_free_space;
    pos_x = x; pos_y = y;
    vel_x = vel_y = acc_x = acc_y = 0.;
    filtered_state_means.push_back(x); filtered_state_means.push_back(y);
    filtered_state_means.push_back(vel_x); filtered_state_means.push_back(vel_y);
    filtered_state_means.push_back(acc_x); filtered_state_means.push_back(acc_y);
    filter->configure(filtered_state_means);
  }

  /*
   * Update our tracked object with new observations
   */
  void update(const std::vector<double>& observations)
  {
    filter->update(observations, filtered_state_means);
    /*
     * Keep track of the distance it's travelled
     * We include an "if" structure to exclude small distance changes,
     * which are likely to have been caused by changes in observation angle
     * or other similar factors, and not due to the object actually moving
     */
    double  delta_dist_travelled = ((pos_x - filtered_state_means[0])^2 +
        (pos_y - filtered_state_means[1])^2)^(1./2.);
    pos_x = filtered_state_means[0];
    pos_y = filtered_state_means[1];
    vel_x = filtered_state_means[2];
    vel_y = filtered_state_means[3];
    acc_x = filtered_state_means[4];
    acc_y = filtered_state_means[5];
  }

private:
  unsigned int id_num;
  ros::Time last_seen;
  bool seen_in_current_scan;
  unsigned int times_seen;
  double confidence;
  double dist_travelled;
  bool is_person;
  bool deleted;
  double in_free_space;
  KalmanFilter* filter;
  std::vector<double> filtered_state_means;
  double pos_x, pos_y;
  double vel_x, vel_y;
  double acc_x, acc_y;
};

/*
 * Tracker for tracking all the people and objects
 */
class KalmanMultiTracker
{
public:
  KalmanMultiTracker(ros::NodeHandle nh) : : nh_(nh), tfListener(tfBuffer)
  {
    prev_track_marker_id = 0;
    prev_person_marker_id = 0;
    new_local_map_received = true;
    max_cost = 9999999;

    fixed_frame = rospy.get_param("fixed_frame", "odom")
    max_leg_pairing_dist = rospy.get_param("max_leg_pairing_dist", 0.8)
    confidence_threshold_to_maintain_track = rospy.get_param("confidence_threshold_to_maintain_track", 0.1)
    publish_occluded = rospy.get_param("publish_occluded", True)
    publish_people_frame = rospy.get_param("publish_people_frame", fixed_frame)
    use_scan_header_stamp_for_tfs = rospy.get_param("use_scan_header_stamp_for_tfs", False)
    publish_detected_people = rospy.get_param("display_detected_people", False)
    dist_travelled_together_to_initiate_leg_pair = rospy.get_param("dist_travelled_together_to_initiate_leg_pair", 0.5)
    scan_topic = rospy.get_param("scan_topic", "scan");
    scan_frequency = rospy.get_param("scan_frequency", 7.5)
    in_free_space_threshold = rospy.get_param("in_free_space_threshold", 0.06)
    confidence_percentile = rospy.get_param("confidence_percentile", 0.90)
    max_std = rospy.get_param("max_std", 0.9)

    //mahalanobis_dist_gate = scipy.stats.norm.ppf(1.0 - (1.0-confidence_percentile)/2., 0, 1.0)
    mahalanobis_dist_gate = 1.6448536269514722;
    max_cov = max_std^2
    //latest_scan_header_stamp_with_tf_available = rospy.get_rostime()

    // ROS publishers
    people_tracked_pub = nh_.advertise<PersonArray> ("people_tracked", 300);
    people_detected_pub = nh_.advertise<PersonArray> ("people_detected", 300);
    marker_pub = nh_.advertise<visualization_msgs::Marker> ("visualization_marker", 300);
    //non_leg_clusters_pub = rospy.Publisher('non_leg_clusters', LegArray, queue_size=300)

    // ROS subscribers
    //detected_clusters_sub = rospy.Subscriber('detected_leg_clusters', LegArray, detected_clusters_callback)
    //local_map_sub = nh_.subscribe<OccupancyGrid>("local_map", 1, &KalmanMultiTracker::local_map_callback, this)
  }
  // void local_map_callback(OccupancyGrid map)
  // {
  //   local_map = map;
  //   new_local_map_received = true;
  // }

  // Determine the degree to which the position (x,y) is in freespace according to our local map
  // double how_much_in_free_space(double x, double y)
  // {
  //   if (local_map not defined) { return in_free_space_threshold * 2; }
  //
  //   // Get the position of (x,y) in local map coords
  //   int map_x = int(round((x - local_map.info.origin.position.x)/local_map.info.resolution));
  //   int map_y = int(round((y - local_map.info.origin.position.y)/local_map.info.resolution));
  //
  //   // Take the average of the local map's values centred at (map_x, map_y), with a kernal size of <kernel_size>
  //   // If called repeatedly on the same local_map, this could be sped up with a sum-table
  //   int sum = 0
  //   kernel_size = 2;
  //   for (int i = map_x - kernel_size; i <= map_x + kernel_size; ++i) {
  //       for (int j = map_y - kernel_size; j < map_y + kernel_size; ++j) {
  //           if (i + j * local_map.info.height < local_map.data.size()) {
  //               sum += local_map.data[i + j * local_map.info.height];
  //           } else {
  //               # We went off the map! position must be really close to an edge of local_map
  //               return in_free_space_threshold * 2;
  //           }
  //       }
  //   }
  //   percent = sum / (((2. * kernel_size + 1)^2.)*100.)
  //   return percent;
  // }

  void match_detections_to_tracks_GNN(std::vector<ObjectTracked>& tracks,
            std::vector<Point>& meas, std::list<ObjectTracked>& fused)
  {
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
    for(std::vector<Point>::iterator it = meas.begin(); it != meas.end();
        it++, r++) {
         std::vector<ObjectTracked>::iterator it_prev = tracks.begin();
         int c = 0;
         for (; it_prev != tracks.end(); it_prev++, c++) {
           if (it_prev->isPerson() and not it_prev->isInFreeSpace()) {
             matrix(r,c) = max_cost;
           } else {
             double cov = it_prev->getMeasToTrackMatchingCov();
             double mahalanobis_dist = sqrt(((it->x - it_prev->pos_x)^2 +
                                             (it->y - it_prev->pos_y)^2) / cov);
             if (mahalanobis_dist < mahalanobis_dist_gate) {
               matrix(r,c) = mahalanobis_dist;
             } else {
               matrix(r,c) = max_cost;
             }
           }
         }
    }

    Munkres<double> m;
    m.solve(matrix);

    // Use the assignment to update the old tracks with new blob measurement
    r = 0;
    std::vector<Point>::iterator it = meas.begin();
    for(int r = 0; r < rows; r++) {
         std::vector<ObjectTracked>::iterator it_prev = tracks.begin();
         for (int c = 0; c < cols; c++) {
              if (matrix(r,c) == 0) {

                   if (r < meas_count && c < tracks_count) {
                        // Does the measurement fall within 3 std's of
                        // the track?
                        if (it_prev->tracker().is_within_region(it->position(),3)) {
                             // Found an assignment. Update the new measurement
                             // with the track ID and age of older track. Add
                             // to fused list
                             //it->matched_track(*it_prev);
                             it_prev->set_measurement(*it);
                             fused.push_back(*it_prev);
                        } else {
                             // TOO MUCH OF A JUMP IN POSITION
                             // Probably a missed track or a new track
                             it_prev->missed();
                             fused.push_back(*it_prev);

                             // And a new track
                             fused.push_back(Entity(next_available_id(),*it));
                        }
                   } else if (r >= meas_count) {
                        it_prev->missed();
                        fused.push_back(*it_prev);
                   } else if (c >= tracks_count) {
                        // Possible new track
                        fused.push_back(Entity(next_available_id(),*it));
                   }
                   break; // There is only one assignment per row
              }
              if (c < tracks_count-1) {
                   it_prev++;
              }
         }
         if (r < meas_count-1) {
              it++;
         }
    }
  }

private:
  ros::NodeHandle nh_;
  objects_tracked;
  potential_leg_pairs;
  potential_leg_pair_initial_dist_travelled;
  people_tracked;
  unsigned int prev_track_marker_id;
  unsigned int prev_person_marker_id;
  ros::Time prev_time;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  //OccupancyGrid local_map;
  bool new_local_map_received;

  std::string fixed_frame;
  double max_leg_pairing_dist;
  double confidence_threshold_to_maintain_track;
  bool publish_occluded;
  std::string publish_people_frame;
  bool publish_detected_people;
  double dist_travelled_together_to_initiate_leg_pair;
  std::string scan_topic;
  double in_free_space_threshold;
  double confidence_percentile;
  double max_std;
  double mahalanobis_dist_gate;
  double max_cov;

  double max_cost;

  ros::Publisher people_tracked_pub;
  ros::Publisher people_detected_pub;
  ros::Publisher marker_pub;

  ros::Subscriber local_map_sub;
};
