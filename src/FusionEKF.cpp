#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
    H_laser_ << 1, 0, 0, 0,
    0, 1, 0, 0;
    noise_ax = 9.0;
    noise_ay = 9.0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
      double px;
      double py;
    // first measurement
    cout << "EKF: " << endl;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar measurements from polar to cartesian coordinates and initialize state.
      */
        double phi = measurement_pack.raw_measurements_[1];
        px = measurement_pack.raw_measurements_[0] * cos(phi);
        py = measurement_pack.raw_measurements_[0] * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
        px = measurement_pack.raw_measurements_[0];
        py = measurement_pack.raw_measurements_[1];
    }

    // done initializing, no need to predict or update
    ekf_.x_ = VectorXd(4);
    ekf_.x_<< px, py, 0, 0;
      
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.setF(dt);
    ekf_.setQ(dt, noise_ax, noise_ay);

    ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        Hj_=tools.CalculateJacobian(ekf_.x_);
        ekf_.R_=R_radar_;
        ekf_.H_=Hj_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        
    } else {
        ekf_.R_=R_laser_;
        ekf_.H_=H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
