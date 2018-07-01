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
  H_radar_ = MatrixXd(3, 4);
  Hj_ = MatrixXd(3, 4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);

  /**
   * Finish initializing the FusionEKF.
   */
  // Init H, measurement function
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;
  H_radar_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // Init R, measurement covariance
  R_laser_ << 0.0225, 0, 0, 0.0225;
  R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  // Init P state covariance matrix
  ekf_.P_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

  // Init F process model function
  ekf_.F_ << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "First measurement RADAR" << endl;
      /**
       Convert radar from polar to cartesian coordinates and initialize state.
       */
      double rho = measurement_pack.raw_measurements_[0];  // range
      double phi = measurement_pack.raw_measurements_[1];  // bearing
      double rho_dot = measurement_pack.raw_measurements_[2];  // velocity of rho

      // Coordinates convertion from polar to cartesian
      double x = rho * cos(phi);
      if (x < 0.0001) {
        x = 0.0001;
      }

      double y = rho * sin(phi);
      if (y < 0.0001) {
        y = 0.0001;
      }

      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);

      ekf_.x_ << x, y, vx, vy;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
       Initialize state.
       */
      // No velocity and coordinates are cartesian already.
      cout << "First measurement LASER" << endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack
          .raw_measurements_[1], 0, 0;
    }


    // Saving first timestamp in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   * Update the state transition matrix F according to the new elapsed time.
   - Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  //1. Modify the F process model matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //2. Set the process noise covariance matrix Q
  double noise_ax = 9.0;
  double noise_ay = 9.0;

  double dt_2 = dt * dt;  //dt^2
  double dt_3 = dt_2 * dt;  //dt^3
  double dt_4 = dt_3 * dt;  //dt^4

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0, 0, dt_4 / 4
      * noise_ay, 0, dt_3 / 2 * noise_ay, dt_3 / 2 * noise_ax, 0, dt_2
      * noise_ax, 0, 0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

  //3. Call the Kalman Filter predict() function
  ekf_.Predict();

  //cout << "xp_ = " << ekf_.x_ << endl;
  //cout << "Pp_ = " << ekf_.P_ << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
