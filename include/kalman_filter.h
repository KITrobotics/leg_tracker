#ifndef FILTERS_KALMAN_FILTER_H
#define FILTERS_KALMAN_FILTER_H

#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <boost/scoped_ptr.hpp>
#include "filters/filter_base.h"
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <Eigen/Dense>


namespace filters {

template <typename T>
class KalmanFilter : public MultiChannelFilterBase <T>
{
public:
  KalmanFilter();
  KalmanFilter(
      double dt,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P,
      const Eigen::VectorXd& x0
  );
  ~KalmanFilter();
  virtual bool configure();
  virtual bool update( const std::vector<T>& data_in, std::vector<T>& data_out);
  
private:
  uint32_t number_of_elements_; ///< Number of elements per observation
    // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};



template <typename T>
KalmanFilter<T>::KalmanFilter()/*:number_of_elements_(0)*/
{}

template <typename T>
KalmanFilter<T>::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P,
    const Eigen::VectorXd& x0
    ) : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(x0), x_hat_new(n)
{
    I.setIdentity();
}

template <typename T>
bool KalmanFilter<T>::configure()
{ 
//   if (!FilterBase<T>::getParam("number_of_elements_", number_of_elements_))
//   {
//     ROS_ERROR("KalmanFilter did not find param number_of_elements_");
//     return false;
//   }
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
  return true;
}

template <typename T>
KalmanFilter<T>::~KalmanFilter()
{}

template <typename T>
bool KalmanFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out)
{
//   if (data_in.size() != number_of_elements_ || data_out.size() != number_of_elements_)
//   {
//     ROS_ERROR("Configured with wrong size config:%d in:%d out:%d", number_of_elements_, (int)data_in.size(), (int)data_out.size());
//     return false;
//   }
  
  
  if(!initialized) { ROS_ERROR("Filter is not initialized!"); return false; }
  
  Eigen::VectorXd y(data_in.size()/*data_in.data()*/);
  for (int i = 0; i < y.size(); ++i) {
    y[i] = data_in[i];
  }
  
  
  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  t += dt;
  
  data_out.resize(x_hat.size()/*data_in.data()*/);
  for (int i = 0; i < x_hat.size(); ++i) {
    data_out[i] = x_hat[i];
  }
  
  
//   data_out(x_hat.data(), x_hat.data() + x_hat.rows() * x_hat.cols());

//   data_out.resize(x_hat.size());
//   Eigen::VectorXd::Map(&data_out[0], x_hat.size()) = x_hat;
  //data_out = data_in + 1;
  return true;
};

}
#endif

