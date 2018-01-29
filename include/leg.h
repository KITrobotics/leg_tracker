
#ifndef LEG_TRACKER_LEG_H
#define LEG_TRACKER_LEG_H

#include <iirob_filters/kalman_filter.h>
#include <pcl/point_types.h>
#include <list>


typedef pcl::PointXYZ Point;
typedef iirob_filters::MultiChannelKalmanFilter<double> KalmanFilter;


int occluded_dead_age = 10;
double var_obs = 0.25;

int min_predictions = 4;
int min_observations = 4;
int state_dimensions = 6;

class Leg
{

// const static int z_coord = 0.178;



private:
  unsigned int legId;
  unsigned int peopleId;
  KalmanFilter* filter;
  Point pos;
  Point vel, acc;
  //std::vector<double> state;
  int predictions;
  int observations;
  bool hasPair_;
  // int min_predictions;
  // int min_observations;
  // int state_dimensions;
  std::list<std::vector<double> > history;
//   bool isRemoved;
  int occluded_age;

public:
  Leg() {}

  Leg(unsigned int id, const Point& p)
  {
    legId = id;
    occluded_age = 0;
    configure(p);
  }

  bool configure(int dimensions, int min_preds, int min_obs, const Point& p)
  {
  //   state_dimensions = dimensions;
  //   min_predictions = min_preds;
  //   min_observations = min_obs;

    peopleId = -1;
    pos = p;
    hasPair_ = false;
    predictions = observations = 0;

    std::vector<double> in;
    // position
    in.push_back(p.x); in.push_back(p.y);
    // velocity
    in.push_back(0.0); in.push_back(0.0);
    // acceleration
    in.push_back(0.0); in.push_back(0.0);

    filter = new KalmanFilter();
    bool result = filter->configure(in);
    if (!result) { ROS_ERROR("Leg.h: Configure of filter has failed!"); }
    return result;
  }

  bool configure(const Point& p)
  {
    peopleId = -1;
    pos = p;
    hasPair_ = false;
    predictions = observations = 0;

    std::vector<double> in;
    // position
    in.push_back(p.x); in.push_back(p.y);
    // velocity
    in.push_back(0.0); in.push_back(0.0);
    // acceleration
    in.push_back(0.0); in.push_back(0.0);

    filter = new KalmanFilter();
    bool result = filter->configure(in);
    if (!result) { ROS_ERROR("Leg.h: Configure of filter has failed!"); }
    return result;
  }

  bool is_within_region(const Point& p, double std)
  {
    Eigen::VectorXd in(2);
    in << p.x, p.y;
    Eigen::VectorXd state(2);
    state << pos.x, pos.y;
    Eigen::MatrixXd B;
    if (!filter->getGatingMatrix(B)) { return false; }
    Eigen::VectorXd diff = in - state;
    Eigen::MatrixXd dist_mat = diff.transpose()*B.inverse()*diff;
    double dist = dist_mat(0,0);
    //if (dist <= pow(nsigma,2)) {
                ROS_WARN("5!");
    if (dist <= std) {
         return true;
    } else {
         return false;
    }
  }

  void missed()
  {
    occluded_age++;
  }

  double getMeasToTrackMatchingCov()
  {
    double result = 0.;
    Eigen::MatrixXd cov;
    if (!filter->getCovarianceMatrix(cov)) { ROS_ERROR("Leg.h: getCovarianceMatrix() has failed!"); return result; }
    result = cov(0, 0);
    result += var_obs;
    return result;
  }

  bool is_dead()
  {
      if (occluded_age > occluded_dead_age) {
           return true;
      }
      return false;
  }

  Point computePrediction()
  {
    std::vector<double> prediction;
    filter->computePrediction(prediction);
    Point p;
    if (prediction.size() == state_dimensions) { p.x = prediction[0]; p.y = prediction[1]; }
    return p;
  }

  void predict()
  {
    std::vector<double> prediction;
    filter->predict(prediction);
    if (prediction.size() != state_dimensions) { ROS_ERROR("Leg.h: Prediction vector size is too small!"); return; }
    pos.x = prediction[0];
    pos.y = prediction[1];
    vel.x = prediction[2];
    vel.y = prediction[3];
    acc.x = prediction[4];
    acc.y = prediction[5];
    if (predictions < min_predictions) { predictions++; }
    observations = 0;
    //state = prediction;
    updateHistory(prediction);
  }

  void update(const Point& p)
  {
    std::vector<double> in, out;
    in.push_back(p.x); in.push_back(p.y);
    filter->update(in, out);
  }

  void update(const std::vector<double>& in, std::vector<double>& out)
  {
    filter->update(in, out);
    if (out.size() != state_dimensions) { ROS_ERROR("Leg.h: Update out vector size is too small!"); return; }
    pos.x = out[0];
    pos.y = out[1];
    vel.x = out[2];
    vel.y = out[3];
    acc.x = out[4];
    acc.y = out[5];
    predictions = 0;
    if (observations < min_observations) { observations++; }
    //state = out;
    updateHistory(out);
  }

  void updateHistory(std::vector<double> new_state)
  {
    if (history.size() >= min_observations) {
      history.pop_front();
    }
    history.push_back(new_state);
  }

  std::list<std::vector<double> > getHistory()
  {
    return history;
  }

  int getPredictions()
  {
    return predictions;
  }

  Point getPos()
  {
    return pos;
  }

  Point getVel()
  {
    return vel;
  }

  int getPeopleId()
  {
	  return peopleId;
  }

  int getObservations()
  {
	  return observations;
  }

  void setPeopleId(int id)
  {
	  peopleId = id;
  }

  void setHasPair(bool value)
  {
	  hasPair_ = value;
  }

  bool hasPair()
  {
	  return hasPair_;
  }

  bool getGatingMatrix(Eigen::MatrixXd& data_out)
  {
    if (!filter->getGatingMatrix(data_out)) { return false; }
    return true;
  }

//   bool likelihood(const double& x, const double& y, double& out)
  double likelihood(const double& x, const double& y)
  {
    std::vector<double> in;
    in.push_back(x); in.push_back(y);
    double out = 0.;
    if (!filter->likelihood(in, out)) { ROS_ERROR("Leg.h: Likelihood failed!"); return false; }
    return out;
  }

};


#endif
