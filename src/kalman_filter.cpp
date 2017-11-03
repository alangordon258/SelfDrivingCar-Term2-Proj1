#include "kalman_filter.h"
#include "tools.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.
const double LARGE_VAR = 999.0;      // large variance
KalmanFilter::KalmanFilter() {
    
    x_ = VectorXd(4);
    
    F_ = MatrixXd(4, 4);
    F_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    Q_ = MatrixXd(4, 4);
    
    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, LARGE_VAR, 0,
    0, 0, 0, LARGE_VAR;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
    
}
void KalmanFilter::setF(float dt) {
    F_(0, 2) = dt;
    F_(1, 3) = dt;
}

void KalmanFilter::setQ(float dt, float noise_ax, float noise_ay) {
    float dt2 = dt * dt;
    float dt3 = dt * dt2;
    float dt4 = dt * dt3;
    
    Q_ << noise_ax * dt4/4, 0, noise_ax*dt3/2, 0,
    0, noise_ay*dt4/4, 0, noise_ay*dt3/2,
    noise_ax * dt3/2, 0, noise_ax*dt2, 0,
    0, noise_ay*dt3/2, 0, noise_ay*dt2;
}
void KalmanFilter::Predict() {
  // Perform Kalman Filter predict step
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    // Perform Kalman Filter update step
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    // New estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    VectorXd z_pred = tools.CartesianToPolar(x_);
    // Perform Kalman Filter update step
    VectorXd y = z - z_pred;
    y[1] = tools.NormalizeAngle(y[1]);
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    // New estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
