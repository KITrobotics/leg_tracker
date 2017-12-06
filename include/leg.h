
#include <iirob_filters/kalman_filter.h>
#include <pcl/point_types.h>

class Leg
{
	
static int id_counter = 0;
	
	
private:
  int peopleId;
  iirob_filters::MultiChannelKalmanFilter<double>* filter;
  pcl::PointXYZ pos;
  int predictions;
  int observations;
  bool hasPair;

public:
  Leg() {}
  bool configure()
  {
    peopleId = -1;
    pos.x = pos.y = pos.z = 0.0;
    predictions = observations = 0;
    filter = new iirob_filters::MultiChannelKalmanFilter<double>();
    bool result = filter->configure();
    if (!result) { ROS_ERROR("Configure of filter has failed!"); }
    return result;
  }
  pcl::PointXYZ computePrediction()
  {
    std::vector<double> prediction;
    filter->computePrediction(prediction);
    pcl::PointXYZ p;
    if (prediction.size() >= 2) { p.x = prediction[0]; p.y = prediction[1]; p.z = 0.0; }
    return p;
  }
  
  void predict()
  {
    std::vector<double> prediction;
    filter->predict(prediction);
    if (prediction.size() >= 2) { pos.x = prediction[0]; pos.y = prediction[1]; }
    predictions++;
  }
  
  void update(const std::vector<double>& in, std::vector<double>& out)
  {
    filter->update(in, out);
    if (out.size() < 2) { ROS_ERROR("Update out vector size is too small!"); return; }
    pos.x = out[0];
    pos.y = out[1];
    predictions = 0;
	observations++;
  }
  
  int getPredictions()
  {
    return predictions;
  }
  
  pcl::PointXYZ getPos()
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
  
  void setPeopleId(Leg* snd)
  {
	  if (peopleId == -1)
	  {
		peopleId = id_counter++;
	  }		  
	  snd->setPeopleId(peopleId);
	  hasPair = true;
	  snd->setHasPair(true);
  }
  
  void setHasPair(bool value)
  {
	  hasPair = value;
  }
  
  bool hasPair()
  {
	  return hasPair;
  }
    
};






