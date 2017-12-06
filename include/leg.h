
#include <iirob_filters/kalman_filter.h>
#include <pcl/point_types.h>

class Leg
{
private:
  int peopleId;
  iirob_filters::MultiChannelKalmanFilter<double>* filter;
  pcl::PointXYZ pos;
  int predictions;

public:
  Leg() {}
  bool configure()
  {
    peopleId = -1;
    pos.x = pos.y = pos.z = 0.0;
    predictions = 0;
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
  }
  
  int getPredictions()
  {
    return predictions;
  }
  
  pcl::PointXYZ getPos()
  {
    return pos;
  }
    
};






