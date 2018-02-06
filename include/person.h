
#ifndef LEG_TRACKER_PERSON_H
#define LEG_TRACKER_PERSON_H

#include <iirob_filters/kalman_filter.h>
#include <leg.h>

class Person
{


private:
  unsigned int peopleId;
  KalmanFilter* filter;
  Point pos;
  int observations;
  int min_observations;
  unsigned int l1, l2;
  int occluded_age;
  int occluded_dead_age;
  double variance_observation;
  std::vector<double> state;
//   bool isRemoved;

public:
  Person(unsigned int peopleId, const Point& pos, int occluded_dead_age = 10,
    double variance_observation = 0.25, int min_observations = 4) 
  {
    this->peopleId = peopleId;
    occluded_age = 0;
    this->pos = pos;
    this->occluded_dead_age = occluded_dead_age;
    this->variance_observation = variance_observation;
    this->min_observations = min_observations;
    observations = 0;
    
    // position
    state.push_back(pos.x); state.push_back(pos.y);
    // velocity
    state.push_back(0.0); state.push_back(0.0);
    // acceleration
    state.push_back(0.0); state.push_back(0.0);

    filter = new KalmanFilter();
    if (!filter->configure(state)) { ROS_ERROR("Leg.h: Configure of filter has failed!"); }
  }
  
  void missed()
  {
    occluded_age++;
    observations = 0;
  }

  bool is_dead()
  {
      if (occluded_age > occluded_dead_age) {
           return true;
      }
      return false;
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

  void predict()
  {
    std::vector<double> prediction;
    filter->predict(prediction);
    pos.x = prediction[0]; 
    pos.y = prediction[1]; 
    state = prediction;
  }

  void update(const Point& p)
  {
    std::vector<double> in, out;
    in.push_back(p.x); in.push_back(p.y);
    filter->update(in, out);
//     if (out.size() != state_dimensions) { ROS_ERROR("Leg.h: Update out vector size is too small!"); return; }
    pos.x = out[0];
    pos.y = out[1];
    state = out;
    if (observations < min_observations) { observations++; }
    occluded_age = 0;
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

  Point getPos()
  {
    return pos;
  }

  unsigned int getPeopleId()
  {
	  return peopleId;
  }

  int getObservations()
  {
	  return observations;
  }

  void setPeopleId(unsigned int id)
  {
	  peopleId = id;
  }

};


#endif