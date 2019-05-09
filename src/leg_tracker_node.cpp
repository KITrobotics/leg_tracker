#include <leg_tracker/leg_tracker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv,"leg_tracker");
  ros::NodeHandle nh("~");
  std::shared_ptr<LegDetector> ld;
  ld.reset(new LegDetector(nh));
  ros::spin();
  return 0;
}
