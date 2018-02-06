
#ifndef LEG_TRACKER_LEG_H
#define LEG_TRACKER_LEG_H

#include <iirob_filters/kalman_filter.h>
#include <pcl/point_types.h>
#include <list>


typedef pcl::PointXYZ Point;
typedef iirob_filters::MultiChannelKalmanFilter<double> KalmanFilter;


int occluded_dead_age = 10;
double variance_observation = 0.25;

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
  int observations;
  bool hasPair_;
  // int min_predictions;
  int min_observations;
  std::list<std::vector<double> > history;
//   bool isRemoved;
  int occluded_age;
  int occluded_dead_age;
  double variance_observation;
  int state_dimensions;

public:
  Leg() = delete;

  Leg(unsigned int legId, const Point& pos, int occluded_dead_age = 10,
    double variance_observation = 0.25, int min_observations = 4,
    int state_dimensions = 6)
  {
    this->legId = legId;
    occluded_age = 0;
    this->pos = pos;
    this->occluded_dead_age = occluded_dead_age;
    this->variance_observation = variance_observation;
    this->min_observations = min_observations;
    this->state_dimensions = state_dimensions;

    peopleId = -1;
    hasPair_ = false;
    observations = 0;

    std::vector<double> in;
    // position
    in.push_back(pos.x); in.push_back(pos.y);
    // velocity
    in.push_back(0.0); in.push_back(0.0);
    // acceleration
    in.push_back(0.0); in.push_back(0.0);

    filter = new KalmanFilter();
    if (!filter->configure(in)) { ROS_ERROR("Leg.h: Configure of filter has failed!"); }
  }

  unsigned int getLegId()
  {
    return legId;
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
    Eigen::MatrixXd dist_mat = diff.transpose() * B.inverse() * diff;
    double dist = dist_mat(0,0);
    if (dist <= std) {
         return true;
    } else {
         return false;
    }
  }

  void missed()
  {
    occluded_age++;
    observations = 0;
  }

  double getMeasToTrackMatchingCov()
  {
    double result = 0.;
    Eigen::MatrixXd cov;
    if (!filter->getCovarianceMatrix(cov)) { ROS_ERROR("Leg.h: getCovarianceMatrix() has failed!"); return result; }
    result = cov(0, 0);
    result += variance_observation;
    return result;
  }

  bool is_dead()
  {
      if (occluded_age > occluded_dead_age) {
           return true;
      }
      return false;
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
    updateHistory(prediction);
  }

  void update(const Point& p)
  {
    std::vector<double> in, out;
    in.push_back(p.x); in.push_back(p.y);
    filter->update(in, out);
    if (out.size() != state_dimensions) { ROS_ERROR("Leg.h: Update out vector size is too small!"); return; }
    pos.x = out[0];
    pos.y = out[1];
    vel.x = out[2];
    vel.y = out[3];
    acc.x = out[4];
    acc.y = out[5];
    if (observations < min_observations) { observations++; }
    history.pop_back(); // remove last prediction becaufe there is an update
    updateHistory(out);
    occluded_age = 0;
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
