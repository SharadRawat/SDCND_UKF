#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  time_us_ = 0.0;

  Xsig_pred_ = Matrix(n_x_,n_aug_);
  weights_ = VectorXd(n_aug_);
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  P_<< 1,0,0,0,0,
        0,1,0,0,0,
        0,0,1,0,0,
        0,0,0,1,0,
        0,0,0,0,1;


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_){
        //initializing x vector
        x_<< 1,1,1,1,1;
        time_us_ = meas_package.timestamp_;
    if (meas_package.sensor_type_== MeasurementPackage::RADAR){
            double rho = meas_package.raw_measurements_[0];
            double phi = meas_package.raw_measurements_[1];
            double rho_dot = meas_package.raw_measurements_[2];
            double x_m = rho*cos(phi);
            double y_m = rho*sin(phi);
            if (x_m < 0.0001){
                x_m = 0.0001;
            }
            if (y_m < 0.0001){
                y_m = 0.0001;
            }
            x_(0) = x_m;
            x_(1) = y_m;

    }
    else if (meas_package.sensor_type_== MeasurementPackage::RADAR){
            x_(0) = meas_package.raw_measurements_[0];
            x_(1) = meas_package.raw_measurements_[1];

    }
      is_initialized_ = true;
  return;
  }
  float delta_t = (meas_package.timestamp_ - time_us_ )/1000000;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  if (meas_package.SensorType == MeasurementPackage::LASER){
    UpdateLidar(meas_package);

    }

    if (meas_package.SensorType == MeasurementPackage::RADAR){
    UpdateRadar(meas_package);

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  /**** Generating Sigma Points
  MatrixXd Xsig = MatrixXd(n_x_, 2*n_x_+1);
  MatrixXd A = P.llt().matrixL();

  Xsig.col(0) = x_;
  for (double i=0;i<2*n_x_+1;++i){
    Xsig.col(i+1) =  x + sqrt(lambda_+ n_x_)*A.col(i);
    Xsig.col(i+1 + n_x_) = x - sqrt(lambda_+ n_x_)*A.col(i);
  }*/

  /****** Generating Aug sigma Points *******/
    MatrixXd Xsig_aug = MatrixXd::Zeros(n_aug_, 2*n_aug_+1);
    MatrixXd A = P.llt().matrixL();
    VectorXd x_aug_ = VectorXd::Zero(7);
    x_aug_.head(n_x_) = x_;

    MatrixXd Q_ = MatrixXd::Zero(2,2);
    Q_ << std_a_*std_a_,0,0,std_yawdd_*std_yawdd_;
    MatrixXd P_aug_ = MatrixXd::Zeros(n_aug_,n_aug_);
    P_aug_.topLeftCorner(n_x_,n_x_) = P_;
    P_aug_.bottomRightcorner((n_aug_-n_x_),(n_aug_-n_x_)) = Q;
    MatrixXd L = P_aug_.llt.()matrixL;
    Xsig.col(0) = x__aug;
    for (double i=0;i<2*n_aug_+1;++i){
        Xsig_aug.col(i+1) =  x_aug_ + sqrt(lambda_+ n_aug_)*L.col(i);
        Xsig_aug.col(i+1 + n_aug_) = x_aug_ - sqrt(lambda_+ n_aug_)*L.col(i);
  }
   /* Predicting sigma points */


   for (double i=0;i<2*n_aug_+1;++i){
    double px   = Xsig_aug(0,i);
    double py   = Xsig_aug(1,i);
    double v    = Xsig_aug(2,i);
    double yaw  = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    if (yawd<0.00001){
        Xsig_pred_(0,i) = px + v*cos(yaw)*delta_t;
        Xsig_pred_(1,i) = py + v*sin(yaw)*delta_t;
        Xsig_pred_(2,i) = v;
        Xsig_pred_(3,i) = yaw + yawd*delta_t;
        Xsig_pred_(4,i) = yawd;
    }

    else {
        Xsig_pred_(0,i) = px + v/yawd*((sin(yaw+yawd*delta_t) - sin(yaw));
        Xsig_pred_(1,i) = py + v/yawd*((- cos(yaw+yawd*delta_t) + cos(yaw));
        Xsig_pred_(2,i) = v;
        Xsig_pred_(3,i) = yaw + yawd*delta_t;
        Xsig_pred_(4,i) = yawd;
    }

    double ppx = 0.5*delta_t*delta_t*cos(yaw)*nu_a;
    double ppy = 0.5*delta_t*delta_t*sin(yaw)*nu_a;
    double vx = delta_t*nu_a;
    double phi = 0.5*delta_t*delta_t*nu_yawdd;
    double phix = delta_t*nu_yawdd;

    Xsig_pred_(0,i) = Xsig_pred_(0,i) + ppx;
    Xsig_pred_(1,i) = Xsig_pred_(1,i) + ppy;
    Xsig_pred_(2,i) = Xsig_pred_(2,i) + vx;
    Xsig_pred_(3,i) = Xsig_pred_(3,i) + phi;
    Xsig_pred_(4,i) = Xsig_pred_(4,i) + phix;

}
    /* Finding mean and covariance*/
    weights_(0) = lambda_/(lambda_+n_aug_);
    for (double i=1;i<2*n_aug_+1;++i){
        weights_(i) = lambda_ /2*(lambda_+n_aug_);
    }

    VectorXd x = VectorXd::Zeros(n_x);
    for (double i=0;i<2*n_aug_+1;++i){
        x = x + weights_(i)*Xsig_pred_.col(i);
    }

    MatrixXd P = MatrixXd::Zero(n_x, n_x);
    for (double i=0;i<2*n_aug_+1;++i){
            // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights(i) * x_diff * x_diff.transpose() ;

    }
}
/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */


}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  MatrixXd Zsig = MatrixXd(n_x_,2*n_aug_ + 1);
  for (double i=0;i<2*n_aug_+1;++i){
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v =  Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double v1 = v*cos(yaw);
    double v2 =v*sin(yaw);

    Zsig(0,i) = sqrt(px*px + py*py);
    Zsig(1,i) = atan2(py/px);
    Zsig(2,i) = ( px*v1 + py*v2 )/ sqrt(px*px + py*py);
  }

  VectorXd z_pred = VectorXd::Zeros(5);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  MatrixXd S = MatrixXd::Zeros(3,3);
    for (int i = 0; i < 2 * n_aug + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  MatrixXd R = MatrixXd(3,3);
  R << std_radr_*std_radr_,0,0,
        0,std_radphi_*std_radphi_,0,
        0,0,std_radrd_*std_radrd_;
    S = S +R;

    VectorXd z = VectorXd(3);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

    MatrixXd Tc = MatrixXd::Zeros(n_x_,3);

    for (double i=0;i<2*n_aug_;++i){
        VectorXd xdiff = Xsig_pred_.col(i) - x;
        while (xdiff[3]>M_PI) {xdiff-=2*M_PI;}
        while (xdiff[3]<M_PI) {xdiff+=2*M_PI;}
        VectorXd zdiff = Zsig_pred_.col(i) - a;
        while (zdiff[3]>M_PI) {zdiff-=2*M_PI;}
        while (zdiff[3]<M_PI) {zdiff+=2*M_PI;}
        Tc = Tc + weights_(i)*(xdiff)*zdiff.transpose();
    }
    MatrixXd k = MatrixXd(n_x,3);
    K = Tc*S.inverse();
    x = x + K*(z - z_pred);
    P = P - K*S*K.transpose():
  }

