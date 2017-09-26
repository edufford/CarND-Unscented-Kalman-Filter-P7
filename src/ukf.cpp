#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Constructer: Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // If this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // If this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Initialize state vector
  x_ = VectorXd(5);
  x_.setZero();
  
  // Initialize covariance matrix
  P_ = MatrixXd(5, 5);
  P_.setZero();
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

  // Laser measurement noise standard deviation x position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation y position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // Flag for initialization state to handle first measurement
  is_initialized_ = false;

  // Initialize previous timestamp with dummy zero value until first measurement
  prev_time_us_ = 0;

  // Number of state variables for CTRV motion model
  n_x_ = 5;
  
  // Number of augmented state variables including process noise vector nu
  n_aug_ = 7;
  
  // Number of columns for augmented matrices
  n_aug_col_ = 2*n_aug_+1;
  
  // Lambda value for setting sigma points
  lambda_ = 3-n_aug_;

  // Initialize sigma point weighting vector
  weights_ = VectorXd(n_aug_col_);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  double weight_x = 0.5 / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < n_aug_col_; i++) { // 2n+1 weights
    weights_(i) = weight_x;
  }
  
  // Initialize predicted sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, n_aug_col_);
  Xsig_pred_.setZero();

  // Initial NIS values for each sensor type
  NIS_laser_ = 0.0;
  NIS_radar_ = 0.0;
}

/**
 * Destructor
 */
UKF::~UKF() {}

/**
 * Main fusion Unscented Kalman filter process loop.
 *
 * If first measurement, just initialize parameters based on type of
 * measurement.
 *
 * If already initialized, perform Unscented Kalman filter Predict step with
 * the current dt time step using nonlinear CTRV motion model.
 *
 * Next, perform Update step using linear Kalman filter if LASER or nonlinear
 * Unscented Kalman filter if RADAR measurement.
 *
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
  // Reset filter if timestamp resets (restart dataset) or
  // has a gap > 1000 sec (switch dataset)
  if ((meas_package.timestamp_ < prev_time_us_) ||
      (abs(prev_time_us_ - meas_package.timestamp_) > 1000000000.0)) {
    is_initialized_ = false;
    NIS_laser_ = 0.0;
    NIS_radar_ = 0.0;
  }
  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  
  // Initialize filter if needed
  if (!is_initialized_) {
    if (((use_laser_ == true) &&
         (meas_package.sensor_type_ == MeasurementPackage::LASER)) ||
        ((use_radar_ == true) &&
         (meas_package.sensor_type_ == MeasurementPackage::RADAR))) {
          
      // Initialize previous timestamp with first timestamp for next dt calc
      prev_time_us_ = meas_package.timestamp_;
          
      // Set initial state x values based on measurement type
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        // Convert RADAR from polar to cartesian coordinates
        // px = rho * cos(phi)
        // py = rho * sin(phi)
        // v = yaw = yaw_dot = 0 (unknown)
        float rho = meas_package.raw_measurements_[0];
        float phi = meas_package.raw_measurements_[1];
        x_ << (rho * cos(phi)), (rho * sin(phi)), 0, 0, 0;
        cout << "RADAR initial x_ = \n" << x_ << endl;
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        // Initialize state directly from measured LASER px, py
        // px = measured px
        // py = measured py
        // v = yaw = yaw_dot = 0 (unknown)
        float px = meas_package.raw_measurements_[0];
        float py = meas_package.raw_measurements_[1];
        x_ << px, py, 0, 0, 0;
        cout << "LIDAR initial x_ = \n" << x_ << endl;
      }
      
      // Set initial state covariance matrix P
      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
      
      // Done initializing
      is_initialized_ = true;
    }
    
    // No need to predict or update for initialization step
    return;
  }
  
  /*****************************************************************************
   *  Predict
   ****************************************************************************/
  if (((use_laser_ == true) &&
       (meas_package.sensor_type_ == MeasurementPackage::LASER)) ||
      ((use_radar_ == true) &&
       (meas_package.sensor_type_ == MeasurementPackage::RADAR))) {
  
        // Compute the time elapsed dt between current and previous measurement.
        // dt unit is seconds.
        double dt = (meas_package.timestamp_ - prev_time_us_) / 1000000.0;
        prev_time_us_ = meas_package.timestamp_;
        
        // Do UKF Predict step with current dt
        Prediction(dt);
  }
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if ((use_laser_ == true) &&
      (meas_package.sensor_type_ == MeasurementPackage::LASER)) {

    // Do linear Update step for LASER measurement
    UpdateLidar(meas_package);
  }
  else if ((use_radar_ == true) &&
          (meas_package.sensor_type_ == MeasurementPackage::RADAR)) {

    // Do UKF Update step for RADAR measurement
    UpdateRadar(meas_package);
  }
}

/**
 * Unscented Kalman filter Predict step.
 *
 * Predict the state and covariance to the current timestep using the nonlinear
 * Unscented Kalman filter steps:
 *  1. Choose augmented sigma points (Xsig_aug) using lambda and process noise
 *  2. Predict augmented sigma points (Xsig_pred_) using CTRV motion model
 *  3. Calculate predicted mean state (x_) and covariance (P_) by lambda weights
 *
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  
  /**
   * UKF Predict Step - Part 1 - Choose representative augmented sigma points
   */
  
  // Create augmented mean vector, state covariance, and sigma point matrix
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.setZero();
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.setZero();
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_aug_col_);
  Xsig_aug.setZero();
  
  // Set augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;
  
  // Set augmented covariance matrix
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_*std_a_; // acceleration covariance
  P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_; // yaw_dot_dot covariance
  
  // Calculate square root of P_aug
  MatrixXd P_aug_sqrt = P_aug.llt().matrixL();
  
  // Set augmented sigma point matrix Xsig_aug
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++) {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * P_aug_sqrt.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * P_aug_sqrt.col(i);
  }
  
  //std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
  
  /**
   * UKF Predict Step - Part 2 - Predict sigma points to current timestep
   */

  // Loop through each augmented sigma point column to set Xsig_pred_
  for (int i = 0; i < n_aug_col_; i++) {
    
    // Extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
    
    // Calculate base predicted state values
    double px_p, py_p;
    
    // Switch motion equation to avoid division by zero from yawd
    if (fabs(yawd) > 0.001) {
      // Driving in a curve
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
      // Driving straight
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }
    double v_p = v; // constant velocity
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd; // constant turn rate
    
    // Add process noise to predicted state values
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;
    
    // Write predicted sigma point into Xsig_pred_ column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  
  //std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;
  
  /**
   * UKF Predict Step - Part 3 - Calculate predicted mean state and covariance
   */
  
  // Set predicted state mean from weighted predicted sigma points
  x_.fill(0.0);
  for (int i = 0; i < n_aug_col_; i++) { // iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }
  
  // Set predicted state covariance matrix from weighted predicted sigma points
  P_.fill(0.0);
  for (int i = 0; i < n_aug_col_; i++) { // iterate over sigma points
    
    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalization between [-pi, pi] for yaw
    while (x_diff(3) > M_PI) { x_diff(3) -= 2.*M_PI; }
    while (x_diff(3) < -M_PI) { x_diff(3) += 2.*M_PI; }
    
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  
  //std::cout << "Predicted state" << std::endl;
  //std::cout << x_ << std::endl;
  //std::cout << "Predicted covariance matrix" << std::endl;
  //std::cout << P_ << std::endl;
}

/**
 * LASER measurement linear Kalman filter Update step.
 *
 * Updates the state (x_) and the state covariance matrix (P_) using a laser
 * measurement [px, py] and the standard linear Kalman filter equations.  The
 * Normalized Innovation Squared (NIS) is also calculated for consistency check.
 *
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  
  // Standard linear Kalman filter equations
  VectorXd z = VectorXd(2);
  z(0) = meas_package.raw_measurements_(0); // p_x
  z(1) = meas_package.raw_measurements_(1); // p_y

  MatrixXd H_laser_ = MatrixXd(2, 5);
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;
  
  MatrixXd R_ = MatrixXd(2, 2);
  R_ << (std_laspx_*std_laspx_),                      0,
                             0, (std_laspy_*std_laspy_);
  
  VectorXd z_pred = H_laser_ * x_;
  VectorXd z_diff = z - z_pred; // innovation (residual error)
  MatrixXd S = H_laser_ * P_ * H_laser_.transpose() + R_;
  
  // Calculate Kalman Gain K based on new measurement
  MatrixXd K = P_ * H_laser_.transpose() * S.inverse();
  
  // Update state x and state covariance matrix P using Kalman Gain K
  x_ = x_ + (K * z_diff);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;
  
  // Calculate LASER NIS for consistency check
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  std::cout << "LASER NIS: " << NIS_laser_ << std::endl;
}

/**
 * RADAR measurement nonlinear Unscented Kalman filter Update step.
 *
 * Updates the state (x_) and the state covariance matrix (P_) using a radar
 * measurement in polar coordinates [rho, phi, rho_dot] and the nonlinear
 * Unscented Kalman filter steps:
 *  1. Transform sigma points to measurement space (Zsig)
 *  2. Calculate predicted measurement mean (z_pred) and covariance (S)
 *  3. Calculate Kalman Gain (K) using cross-covariance (Tc)
 *  4. Update state (x_) and covariance (P_) with Kalman Gain
 *
 * The Normalized Innovation Squared (NIS) is also calculated for
 * consistency check.
 *
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  /**
   * UKF Update Step - Part 1 - Transform sigma points to measurement space
   */
  
  // Create matrix for predicted measurement sigma points
  MatrixXd Zsig = MatrixXd(3, n_aug_col_);
  Zsig.setZero();
  
  // Transform predicted sigma points into measurement space
  for (int i = 0; i < n_aug_col_; i++) { // 2n+1 sigma points
    
    // extract values for better readibility
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double vx = v * cos(yaw);
    double vy = v * sin(yaw);
    
    // Measurement model
    Zsig(0,i) = sqrt(px*px + py*py); // rho
    Zsig(1,i) = atan2(py, px); // phi
    Zsig(2,i) = (px*vx + py*vy) / (sqrt(px*px + py*py)); // rho_dot
  }
  
  /**
   * UKF Update Step - Part 2 - Calculate predicted measurement mean, covariance
   */
  
  // Set predicted measurement mean from weighted predicted measurement
  // sigma points
  VectorXd z_pred = VectorXd(3);
  z_pred.fill(0.0);
  for (int i=0; i < n_aug_col_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(3, 3);
  S.fill(0.0);
  for (int i = 0; i < n_aug_col_; i++) { // 2n+1 sigma points
    
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    // Angle normalization between [-pi, pi] for phi
    while (z_diff(1) > M_PI) { z_diff(1) -= 2.*M_PI; }
    while (z_diff(1) < -M_PI) { z_diff(1) += 2.*M_PI; }
    
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  // Add measurement noise covariance matrix R
  MatrixXd R = MatrixXd(3, 3);
  R << (std_radr_*std_radr_),                         0,                      0,
                           0, (std_radphi_*std_radphi_),                      0,
                           0,                         0, (std_radrd_*std_radrd_);

  S = S + R;
  
  //std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  //std::cout << "S: " << std::endl << S << std::endl;

  /**
   * UKF Update Step - Part 3 - Calculate Kalman Gain using cross-covariance
   */
  
  // Create state-measurement cross-covariance matrix Tc
  MatrixXd Tc = MatrixXd(n_x_, 3);
  Tc.fill(0.0);
  for (int i = 0; i < n_aug_col_; i++) { // 2n+1 sigma points
    
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    // Angle normalization between [-pi, pi] for phi
    while (z_diff(1) > M_PI) { z_diff(1) -= 2.*M_PI; }
    while (z_diff(1) < -M_PI) { z_diff(1) += 2.*M_PI; }
    
    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    // Angle normalization between [-pi, pi] for yaw
    while (x_diff(3) > M_PI) { x_diff(3) -= 2.*M_PI; }
    while (x_diff(3) < -M_PI) { x_diff(3) += 2.*M_PI; }
    
    // Cross-covariance matrix Tc
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  // Calculate Kalman Gain K
  MatrixXd K = Tc * S.inverse();
  
  /**
   * UKF Update Step - Part 4 - Update state and covariance with Kalman Gain
   */
  
  // Residual between actual and predicted measurement
  VectorXd z = VectorXd(3);
  z(0) = meas_package.raw_measurements_(0); // rho
  z(1) = meas_package.raw_measurements_(1); // phi
  z(2) = meas_package.raw_measurements_(2); // rho_dot

  VectorXd z_diff = z - z_pred; // innovation (residual error)
  
  // Angle normalization between [-pi, pi] for phi
  while (z_diff(1) > M_PI) { z_diff(1) -= 2.*M_PI; }
  while (z_diff(1) < -M_PI) { z_diff(1) += 2.*M_PI; }
  
  // Update state mean and covariance matrix with Kalman Gain
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  
  //std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  //std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
  
  // Calculate RADAR NIS for consistency check
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  std::cout << "RADAR NIS: " << NIS_radar_ << std::endl;
}
