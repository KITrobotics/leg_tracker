
#include <iirob_filters/kalman_filter.h>
#include <leg.h>

class Person
{

// const static int z_coord = 0.178;


private:
  int peopleId;
  KalmanFilter* filter;
  Point pos;
  int predictions;
  int observations;
  int min_predictions;
  int min_observations;
  Leg* left, right;
//   bool isRemoved;

public:
  Person(int min_preds, int min_obs) : min_predictions(min_preds), min_observations(min_obs) {}

  bool configure(Point p, Leg* l, Leg* r)
  {
    peopleId = -1;
    pos = p;
    predictions = observations = 0;
    left = l;
    right = r;

    std::vector<double> in;
    // position
    in.push_back(p.x); in.push_back(p.y);
    // velocity
    in.push_back(0.0); in.push_back(0.0);
    // acceleration
    in.push_back(0.0); in.push_back(0.0);

    filter = new KalmanFilter();
    bool result = filter->configure(in);
    if (!result) { ROS_ERROR("Configure of filter has failed!"); }
    return result;
  }

  Point computePrediction()
  {
    std::vector<double> prediction;
    filter->computePrediction(prediction);
    Point p;
    if (prediction.size() >= 2) { p.x = prediction[0]; p.y = prediction[1]; }
    return p;
  }

  void predict()
  {
    std::vector<double> prediction;
    filter->predict(prediction);
    if (prediction.size() >= 2) { pos.x = prediction[0]; pos.y = prediction[1]; }
    if (predictions < min_predictions) { predictions++; }
    observations = 0;
  }

  void update(const std::vector<double>& in, std::vector<double>& out)
  {
    filter->update(in, out);
    if (out.size() < 2) { ROS_ERROR("Update out vector size is too small!"); return; }
    pos.x = out[0];
    pos.y = out[1];
    predictions = 0;
    if (observations < min_observations) { observations++; }
  }

  int getPredictions()
  {
    return predictions;
  }

  Point getPos()
  {
    return pos;
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

  bool likelihood(const double& x, const double& y, double& out)
  {
    std::vector<double> in;
    in.push_back(x); in.push_back(y);

    if (!filter->likelihood(in, out)) { return false; }
    return true;
  }

};
