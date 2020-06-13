#include <iostream>

#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
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

  /*
  // Student BAUDISCH: original values from framework.
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  */

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30; // TODO: looks way too much

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30; // TODO: is this okay?
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // Student BAUDISCH - Start
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // finish x_ and P_ initialization
  x_.fill( 0.0 );
  P_.fill( 0.0 );

  is_initialized_ = false;

  // State dimension
  n_x_ = 5;

  // Radar measurement dimension
  n_z_ = 3;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Weights of sigma points
  weights_ = VectorXd( 2 * n_aug_ + 1 );
  double w = 1.0 / ( lambda_ + n_aug_ );
  weights_ = VectorXd::Constant( 2 * n_aug_ + 1, 0.5 * w );
  weights_(0) = lambda_ * w;

  // predicted sigma points matrix
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill( 0.0 );

  // augmented sigma covariance matrix
  Xsig_aug_ = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug_.fill( 0.0 );

  // time when the state is true, in us
  time_us_ = 0;

  // measurement covariance
  R_ = MatrixXd(2, 2);
  R_ << 0.0225, 0,
            0, 0.0225;

  // measurement matrix
  H_ = MatrixXd(2, 4);
  H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  // the initial transition matrix F_
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  /// Student BAUDISCH - End
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  
  
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  /*
  long meas_package.timestamp_;
  enum meas_package.sensor_type_ = { LASER, RADAR };
  Eigen::VectorXd meas_package.raw_measurements_;
  */
  if( is_initialized_ )
  {
    double delta_t = meas_package.timestamp_ - time_us_;
    Prediction(delta_t);
    if( meas_package.sensor_type_ == MeasurementPackage::LASER )
    {
      UpdateLidar( meas_package );
      /*
      // 1. Modify the F matrix so that the time is integrated
      F_(0,2) = delta_t;
      F_(1,3) = delta_t;
      // 2. Set the process covariance matrix Q
      double dt2  = delta_t * delta_t;
      double dt3h = dt2 * delta_t;
      double dt4q = dt3h * delta_t;
      dt3h /= 2.0;
      dt4q /= 4.0;
      Q_ = MatrixXd(4, 4);
      Q_ << dt4q * noise_ax, 0.0, dt3h * noise_ax, 0.0,
               0.0, dt4q * noise_ay, 0.0, dt3h * noise_ay,
               dt3h * noise_ax, 0.0, dt2 * noise_ax, 0.0,
               0.0, dt3h * noise_ay, 0.0, dt2 * noise_ay;
    */
    }
    else
    {
      UpdateRadar( meas_package );
    }
  }
  else
  {
    if( meas_package.sensor_type_ == MeasurementPackage::LASER )
    {
      x_ << meas_package.raw_measurements_[0],
            meas_package.raw_measurements_[1],
            0.0,
            0.0;
      // UpdateLidar( meas_package );
    }
    else
    {
      UpdateRadar( meas_package );
    }
  }
  time_us_ = meas_package.timestamp_;
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  // given:
  //     x_ = VectorXd(5)
  //     P_ = MatrixXd(5, 5);
  GenerateAugmentedSigmaPoints();
  SigmaPointPrediction( delta_t );
  PredictMeanAndCovariance();
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  VectorXd z = meas_package.raw_measurements_;

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

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  VectorXd z;
  MatrixXd Zsig;
  VectorXd z_pred;
  MatrixXd S;
  z = meas_package.raw_measurements_;
  PredictRadarMeasurement(z, Zsig, z_pred, S);
  UpdateState(z, Zsig, z_pred, S);
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::GenerateAugmentedSigmaPoints()
{
  // create augmented mean state
  VectorXd x_aug = VectorXd::Zero( n_aug_ );
  x_aug.head( n_x_ ) = x_;

  // create augmented covariance matrix
  // create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero( n_aug_, n_aug_ );
  P_aug.topLeftCorner( n_x_, n_x_ ) = P_;
  P_aug( n_x_    , n_x_     ) = std_a_ * std_a_;
  P_aug( n_x_ + 1, n_x_ + 1 ) = std_yawdd_ * std_yawdd_;

  // calculate square root of P
  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  A *= sqrt( lambda_ + static_cast<double>( n_aug_ ) );

  // create augmented sigma points
  Xsig_aug_ = MatrixXd( n_aug_, 2 * n_aug_ + 1 );
  for( int col = 0; col < 2 * n_aug_ + 1; col++ )
  {
    Xsig_aug_.block( 0, col, n_aug_, 1) = x_aug;
  }
  Xsig_aug_.block( 0,          1, n_aug_, n_aug_) += A;
  Xsig_aug_.block( 0, 1 + n_aug_, n_aug_, n_aug_) -= A;
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::SigmaPointPrediction(double delta_t)
{
  // predict sigma points
  for( int i = 0; i < Xsig_pred_.cols(); i++)
  {
    VectorXd x_k = Xsig_aug_.block( 0, i, n_x_, 1 );
    double px            = x_k[ 0 ];
    double py            = x_k[ 1 ];
    double v             = x_k[ 2 ];
    double psi           = x_k[ 3 ];
    double psiD          = x_k[ 4 ];
    double upsilon_a     = Xsig_aug_( 5, i );
    double upsilon_psiDD = Xsig_aug_( 6, i );

    // avoid division by zero
    VectorXd p1 = VectorXd( n_x_ );
    if( fabs( psiD ) <= 1.0e-10 ) // check for very small values instead for == 0.0
    {
        p1 << v * cos( psi ) * delta_t,
              v * sin( psi ) * delta_t,
              0.0,
              psiD * delta_t,
              0.0;
    }
    else
    {
        p1 << v / psiD * ( sin( psi + psiD * delta_t ) - sin( psi ) ),
              v / psiD * (-cos( psi + psiD * delta_t ) + cos( psi ) ),
              0.0,
              psiD * delta_t,
              0.0;
    }
    VectorXd p2 = VectorXd( n_x_ );
    p2 << 0.5 * delta_t * delta_t * cos( psi ) * upsilon_a,
          0.5 * delta_t * delta_t * sin( psi ) * upsilon_a,
          delta_t * upsilon_a,
          0.5 * delta_t * delta_t * upsilon_psiDD,
          delta_t * upsilon_psiDD;
    

    VectorXd x_kp1 = x_k + p1 + p2;
    //std::cout << "x_kp1 = " << std::endl << x_kp1 << std::endl;

    // write predicted sigma points into right column
    Xsig_pred_.col(i) = Xsig_aug_.block( 0, i, n_x_, 1 ) + p1 + p2;
  }
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::PredictMeanAndCovariance()
{
  assert( Xsig_pred_.rows() == n_x_ );
  assert( Xsig_pred_.cols() == 2 * n_aug_ + 1 );

  // predict state mean
  x_.fill( 0.0 );
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    x_ += weights_( i ) * Xsig_pred_.col( i );
  }

  // predict state covariance matrix
  P_.fill( 0.0 );
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd t = Xsig_pred_.col( i ) - x_;

    // angle normalization (taken from lesson 4/24: Predicted mean and covariance solution)
    // Reference: https://classroom.udacity.com/nanodegrees/nd313/parts/da5e72fc-972d-42ae-bb76-fca3d3b2db06/modules/c32eb575-0080-4a25-a087-98ab84cb3092/lessons/daf3dee8-7117-48e8-a27a-fc4769d2b954/concepts/07b59fdd-adb3-479b-8566-336332cf5f09
    while ( t(3) >  M_PI ) t(3) -= 2.0 * M_PI;
    while ( t(3) < -M_PI ) t(3) += 2.0 * M_PI;

    P_ += weights_(i) * ( t * t.transpose().eval() );
  }
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::PredictRadarMeasurement(VectorXd& z_out, MatrixXd &Zsig_out, VectorXd& z_pred, MatrixXd& S_out)
{
  // create matrix for sigma points in measurement space
  Zsig_out = MatrixXd( n_z_, 2 * n_aug_ + 1 );

  // mean predicted measurement
  z_pred = VectorXd( n_z_ );
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd( n_z_, n_z_ );

  // transform sigma points into measurement space
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
      double px   = Xsig_pred_( 0, i );
      double py   = Xsig_pred_( 1, i );
      double nu   = Xsig_pred_( 2, i );
      double psi  = Xsig_pred_( 3, i );
      //double psid = Xsig_pred_( 4, i );

      double lx = sqrt( px * px + py * py );
      if( lx > 1.0e-10 )
      {
        Zsig_out( 0, i ) = lx;
        Zsig_out( 1, i ) = ( fabs( px ) >= 1.0e-8 ) ? atan2( py, px ) : ( (px*py >= 0.0) ? M_PI_2 : -M_PI_2 );
        Zsig_out( 2, i ) = nu * ( px * cos( psi ) + py * sin( psi ) ) / lx;
      }
      else
      {
        Zsig_out( 0, i ) = 0.0;
        Zsig_out( 1, i ) = 0.0;
        Zsig_out( 2, i ) = 0.0;
      }
  }

  // calculate mean predicted measurement
  z_pred.fill( 0.0 );
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
      z_pred += weights_( i ) * Zsig_out.col(i);
  }

  // calculate innovation covariance matrix S
  S.fill( 0.0 );
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
      VectorXd t( n_z_ );
      t = Zsig_out.col(i) - z_pred;
      S += weights_( i ) * ( t * t.transpose().eval() );
  }

  S( 0, 0 ) += std_radr_ * std_radr_;
  S( 1, 1 ) += std_radphi_ * std_radphi_;
  S( 2, 2 ) += std_radrd_ * std_radrd_;

  // write result
  z_out = z_pred;
  S_out = S;
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// Update
void UKF::UpdateState(VectorXd& z_in, MatrixXd &Zsig_in, VectorXd& z_pred, MatrixXd& S_in)
{
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd( n_x_, n_z_ );

  // calculate cross correlation matrix
  Tc.fill( 0.0 );
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
    VectorXd x_diff( n_x_ );
    VectorXd z_diff( n_z_ );
    x_diff = Xsig_pred_.col( i ) - x_;
    z_diff = Zsig_in.col( i ) - z_pred;

    // angle normalization (taken from the solution)
    while ( x_diff( 3 ) >  M_PI ) x_diff( 3 ) -= 2.0 * M_PI;
    while ( x_diff( 3 ) < -M_PI ) x_diff( 3 ) += 2.0 * M_PI;
    // angle normalization (taken from the solution)
    while ( z_diff( 1 ) >  M_PI ) z_diff( 1 ) -= 2.0 * M_PI;
    while ( z_diff( 1 ) < -M_PI ) z_diff( 1 ) += 2.0 * M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose().eval();
  }

  // calculate Kalman gain K;
  MatrixXd K( n_x_, n_z_ );
  K = Tc * S_in.inverse();

  // update state mean and covariance matrix
  VectorXd z_diff( n_z_ );
  z_diff = z_in - z_pred;
  // angle normalization (taken from the solution)
  while ( z_diff( 1 ) >  M_PI ) z_diff( 1 ) -= 2.0 * M_PI;
  while ( z_diff( 1 ) < -M_PI ) z_diff( 1 ) += 2.0 * M_PI;

  x_ += K * z_diff;
  P_ -= K * (S_in * K.transpose());
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
