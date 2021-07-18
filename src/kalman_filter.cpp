#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

/************************************************************
   * Constructor.
************************************************************/
KalmanFilter::KalmanFilter() {}

/*************************************************************
 * Destructor.
*************************************************************/
KalmanFilter::~KalmanFilter() {}

/*************************************************************
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
*************************************************************/
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/**************************************************************
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
**************************************************************/
void KalmanFilter::Predict() {

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

/**************************************************************
  * Updates the state by using standard Kalman Filter equations
  * @param z The measurement at k+1
**************************************************************/
void KalmanFilter::Update(const VectorXd &z) {

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

/**************************************************************
 * Updates the state by using Extended Kalman Filter equations
 * @param z The measurement at k+1
**************************************************************/
void KalmanFilter::UpdateEKF(const VectorXd &z) {

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float rho = sqrt(px*px + py*py);
  float phi = atan2(py,px);
  if (rho < 0.001) {
    rho = 0.001;
  }
  float rhodot = (px*vx + py*vy)/rho;

  VectorXd Hx = VectorXd(3);
  Hx << rho, phi, rhodot;
  VectorXd y = z - Hx;

  while (y[1] < -M_PI){
    y[1] += 2*M_PI;
  }
  while (y[1] > M_PI){
    y[1] -= 2*M_PI;
  }
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
