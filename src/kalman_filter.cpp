#include "kalman_filter.h"
#include "pluginlib/class_list_macros.h"


PLUGINLIB_EXPORT_CLASS(filters::KalmanFilter<double>, filters::MultiChannelFilterBase<double>)