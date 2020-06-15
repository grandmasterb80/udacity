#include <iostream>
#include <sstream>

#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define DEBUG_LEVEL     0
#define UKF_ID          3

#define     DEBUG(level, instr)     do { if(level <= DEBUG_LEVEL) { instr; } } while(0);
#define UKF_DEBUG(level, instr)     do { if(FID==UKF_ID && level <= DEBUG_LEVEL) { instr; } } while(0);

#define     sign(a)     ( ( (a) >= 0.0 ) ? (a) : (-(a)) )

static int ID = 0;

/**
  * Normalize angle (in radiant), i.e. the function will ensure to return a value between -pi and +pi.
  */
double NormalizeAngle( double angle )
{
  // round to zero; offset to get start at one and get an additional counter for every additional 2pi in z_diff(3)
  if( angle >  M_PI ) angle = angle - trunc( ( angle + M_PI ) / ( 2.0 * M_PI ) ) * 2.0 * M_PI;
  // round to zero; offset to get start at one and get an additional counter for every additional 2pi in z_diff(3)
  if( angle < -M_PI ) angle = angle - trunc( ( angle - M_PI ) / ( 2.0 * M_PI ) ) * 2.0 * M_PI;

  assert( angle >= -M_PI );
  assert( angle <=  M_PI );
  return angle;
}

/**
 * Translate vector into string. This function prints everything into a single
 * line, components are separated by a tab.
 */
std::string vec2str(Eigen::VectorXd &x) {
  std::stringstream s;
  s << "<" << x(0);
  for(int i = 1; i < x.size(); i++)
  {
    s << ",\t" << x(i);
  }
  s << ">";
  return s.str();
}

// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
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
  std_a_ = 30; // TODO: looks way too much

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30; // TODO: looks way too much
  */

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // ISO-15622 says 5m/s^2, video says expect max 6m/s^2 and set std_a_ to half of it.
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI / 180.0 * 0.1;
  
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
  // assign unique ID to this filter
  FID = ID;
  ID++;
  DEBUG(1, std::cout << __LINE__ << " UKF with ID " << FID << " initialized" << std::endl );

  is_initialized_ = false;

  // State dimension
  n_x_ = 5;

  // Radar measurement dimension
  n_z_radar_ = 3;
  // Lidar measurement dimension
  n_z_lidar_ = 2;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // finish x_ and P_ initialization
  x_.fill( 0.0 );
  P_ = MatrixXd::Identity( n_x_, n_x_ );

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

  NIS_radar_ = 0.0;
  NIS_lidar_ = 0.0;
  NIS_radar_950_ = 0;
  NIS_radar_900_ = 0;
  NIS_radar_100_ = 0;
  NIS_radar_050_ = 0;
  NIS_radar_total_ = 0;
  NIS_lidar_950_ = 0;
  NIS_lidar_900_ = 0;
  NIS_lidar_100_ = 0;
  NIS_lidar_050_ = 0;
  NIS_lidar_total_ = 0;
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  /// Student BAUDISCH - End
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
}

UKF::~UKF() {
  if( NIS_radar_total_ > 0 )
  {
    DEBUG(1, std::cout << __LINE__ << " NIS radar for FID=" << FID << ": " <<
    "\t\t0.950: " << (double)NIS_radar_950_ / NIS_radar_total_ <<
    "\t\t0.900: " << (double)NIS_radar_900_ / NIS_radar_total_ <<
    "\t\t0.100: " << (double)NIS_radar_100_ / NIS_radar_total_ <<
    "\t\t0.050: " << (double)NIS_radar_050_ / NIS_radar_total_ <<
    "\t\ttotal: " << NIS_radar_total_ <<
    std::endl );
  }

  if( NIS_lidar_total_ > 0 )
  {
    DEBUG(1, std::cout << __LINE__ << " NIS lidar for FID=" << FID << ": " <<
    "\t\t0.950: " << (double)NIS_lidar_950_ / NIS_lidar_total_ <<
    "\t\t0.900: " << (double)NIS_lidar_900_ / NIS_lidar_total_ <<
    "\t\t0.100: " << (double)NIS_lidar_100_ / NIS_lidar_total_ <<
    "\t\t0.050: " << (double)NIS_lidar_050_ / NIS_lidar_total_ <<
    "\t\ttotal: " << NIS_lidar_total_ <<
    std::endl );
  }
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  DEBUG(9, std::cout << __LINE__ << " entering " << __FUNCTION__ << std::endl );
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if( ( meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_ ) ||
      ( meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_ ) ) return;

  /*
  long meas_package.timestamp_;
  enum meas_package.sensor_type_ = { LASER, RADAR };
  Eigen::VectorXd meas_package.raw_measurements_;
  */
  if( is_initialized_ )
  {
    double delta_t = ( meas_package.timestamp_ - time_us_ ) / 1.0e6;
    if( meas_package.sensor_type_ == MeasurementPackage::LASER )
    {
      MatrixXd m = MatrixXd( n_x_, n_x_ );
      //m = MatrixXd::Identity( n_x_, n_x_ );
      m << 0.10, 0.00, 0.00, 0.00, 0.00,
           0.00, 0.10, 0.00, 0.00, 0.00,
           0.00, 0.00, 0.50, 0.00, 0.00,
           0.00, 0.00, 0.00, 0.20, 0.00,
           0.00, 0.00, 0.00, 0.00, 0.10;
      P_ = P_ + m * delta_t;
      DEBUG(9, std::cout << __LINE__ << " Lidar update for UKF " << FID << std::endl );
      UKF_DEBUG( 8, std::cout << __LINE__ << " x_ before prediction " << vec2str( x_ ) << std::endl );
      Prediction( delta_t );
      UKF_DEBUG( 8, std::cout << __LINE__ << " x_ after prediction  " << vec2str( x_ ) << std::endl );
      UpdateLidar( meas_package );
      UKF_DEBUG( 6, std::cout << __LINE__ << " x_ after update      " << vec2str( x_ ) << std::endl );
      UKF_DEBUG( 7, std::cout << __LINE__ << " P_ after update      " << std::endl << P_ << std::endl );
    }
    else
    {
      MatrixXd m = MatrixXd( n_x_, n_x_ );
      //m = MatrixXd::Identity( n_x_, n_x_ );
      m << 0.10, 0.00, 0.00, 0.00, 0.00,
           0.00, 0.10, 0.00, 0.00, 0.00,
           0.00, 0.00, 0.20, 0.00, 0.00,
           0.00, 0.00, 0.00, 0.20, 0.00,
           0.00, 0.00, 0.00, 0.00, 0.10;
      P_ = P_ + m * delta_t;
      DEBUG(9, std::cout << __LINE__ << " Radar update for UKF " << FID << std::endl );
      UKF_DEBUG( 8, std::cout << __LINE__ << " x_ before prediction " << vec2str( x_ ) << std::endl );
      Prediction( delta_t );
      UKF_DEBUG( 8, std::cout << __LINE__ << " x_ after prediction  " << vec2str( x_ ) << std::endl );
      UpdateRadar( meas_package );
      UKF_DEBUG( 6, std::cout << __LINE__ << " x_ after update      " << vec2str( x_ ) << std::endl );
      UKF_DEBUG( 7, std::cout << __LINE__ << " P_ after update      " << std::endl << P_ << std::endl );
    }
  }
  else
  {
    if( meas_package.sensor_type_ == MeasurementPackage::LASER )
    {
      DEBUG(4, std::cout << __LINE__ << " Lidar init for UKF " << FID << std::endl );
      x_ << meas_package.raw_measurements_[0],
            meas_package.raw_measurements_[1],
            0.0,
            0.0,
            0.0;
      P_ << std_laspx_*std_laspx_, 0.0, 0.0, 0.0, 0.0,    // std dev of lidar
            0.0, std_laspy_*std_laspy_, 0.0, 0.0, 0.0,    // std dev of lidar
            0.0, 0.0, 1600.0, 0.0, 0.0,                    // bicycle should not be sooo fast. Let's assume it can have up to 20m/s (would cover even a race bike)
            0.0, 0.0, 0.0, M_PI*M_PI, 0.0,                // maximum deviation of angle will be M_PI
            0.0, 0.0, 0.0, 0.0, (M_PI/2.0)*(M_PI/2.0);    // 36degree/sec as maximum deviation of the rotation speed
    }
    else
    {
      DEBUG(4, std::cout << __LINE__ << " Radar init for UKF " << FID << std::endl );
      double r        = meas_package.raw_measurements_[0];
      double phi      = meas_package.raw_measurements_[1];
      double radial_v = meas_package.raw_measurements_[2];
      x_ << cos( phi ) * r,
            sin( phi ) * r,
            radial_v / cos( phi ),    // assume that vehicle drive in same direction
            0.0,
            0.0;
      P_ << cos( phi ) * std_radr_ * std_radr_ + sin( phi ) * std_radphi_ * std_radphi_, 0.0, 0.0, 0.0, 0.0,
            0.0, sin( phi ) * std_radr_ * std_radr_ - cos( phi ) * std_radphi_ * std_radphi_, 0.0, 0.0, 0.0,
            0.0, 0.0, cos( phi ) * std_radrd_ * std_radrd_, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0;
    }
    is_initialized_ = true;
  }
  time_us_ = meas_package.timestamp_;
  DEBUG(9, std::cout << __LINE__ << " leaving " << __FUNCTION__ << std::endl );
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::Prediction(double delta_t) {
  DEBUG(9, std::cout << __LINE__ << " entering " << __FUNCTION__ << std::endl );
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
  DEBUG(9, std::cout << __LINE__ << " leaving " << __FUNCTION__ << std::endl );
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  DEBUG(9, std::cout << __LINE__ << " entering " << __FUNCTION__ << std::endl );
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // ------------------------ Predict Measurement  ----------------------------
  VectorXd z = meas_package.raw_measurements_.head( n_z_lidar_ );

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd( n_z_lidar_, 2 * n_aug_ + 1 );

  // mean predicted measurement
  VectorXd z_pred = VectorXd( n_z_lidar_ );
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd( n_z_lidar_, n_z_lidar_ );

  // transform sigma points into measurement space
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
    Zsig.col( i ) = Xsig_pred_.col( i ).head( n_z_lidar_ );
  }

  // calculate mean predicted measurement
  z_pred.fill( 0.0 );
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
      z_pred += weights_( i ) * Zsig.col(i);
  }

  // calculate innovation covariance matrix S
  S.fill( 0.0 );
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
      VectorXd t( n_z_lidar_ );
      t = Zsig.col(i) - z_pred;
      S += weights_( i ) * ( t * t.transpose() );
  }

  // add Lidar specific measurement noise
  S( 0, 0 ) += std_laspx_ * std_laspx_;
  S( 1, 1 ) += std_laspy_ * std_laspy_;

  // ------------------------ Update ----------------------------
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd( n_x_, n_z_lidar_ );

  // calculate cross correlation matrix
  Tc.fill( 0.0 );
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
    VectorXd x_diff( n_x_ );
    VectorXd z_diff( n_z_lidar_ );
    x_diff = Xsig_pred_.col( i ) - x_;
    z_diff = Zsig.col( i ) - z_pred;

    x_diff( 3 ) = NormalizeAngle( x_diff( 3 ) );

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd K( n_x_, n_z_lidar_ );
  K = Tc * S.inverse();

  // update state mean and covariance matrix
  VectorXd z_diff( n_z_lidar_ );
  z_diff = z - z_pred;

  x_ += K * z_diff;
  P_ -= K * (S * K.transpose());

  // ------------------------ NIS ----------------------------
  NIS_lidar_ = z_diff.transpose() * S * z_diff;
  if( NIS_lidar_ > 5.991 ) NIS_lidar_050_++;
  if( NIS_lidar_ > 0.103 ) NIS_lidar_950_++;
  if( NIS_lidar_ > 4.605 ) NIS_lidar_100_++;
  if( NIS_lidar_ > 0.211 ) NIS_lidar_900_++;
  NIS_lidar_total_++;

  DEBUG(8, std::cout << __LINE__ << " NIS lidar " << NIS_lidar_ << std::endl );
  DEBUG(9, std::cout << __LINE__ << " leaving " << __FUNCTION__ << std::endl );
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  DEBUG(9, std::cout << __LINE__ << " entering " << __FUNCTION__ << std::endl );
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  MatrixXd Zsig;
  VectorXd z_pred;
  MatrixXd S;
  VectorXd z = meas_package.raw_measurements_.head( n_z_radar_ );
  PredictRadarMeasurement(Zsig, z_pred, S);
  UpdateStateRadar(z, Zsig, z_pred, S);
  DEBUG(9, std::cout << __LINE__ << " leaving " << __FUNCTION__ << std::endl );
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::GenerateAugmentedSigmaPoints()
{
  DEBUG(9, std::cout << __LINE__ << " entering " << __FUNCTION__ << std::endl );
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
  DEBUG(9, std::cout << __LINE__ << " leaving " << __FUNCTION__ << std::endl );
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::SigmaPointPrediction(double delta_t)
{
  DEBUG(9, std::cout << __LINE__ << " entering " << __FUNCTION__ << std::endl );
  // predict sigma points
  for( int i = 0; i < Xsig_pred_.cols(); i++)
  {
    VectorXd x_k = Xsig_aug_.block( 0, i, n_x_, 1 );
    double px            = x_k( 0 );
    double py            = x_k( 1 );
    double v             = x_k( 2 );
    double psi           = x_k( 3 );
    double psiD          = x_k( 4 );
    double upsilon_a     = Xsig_aug_( 5, i );
    double upsilon_psiDD = Xsig_aug_( 6, i );

    // avoid division by zero
    VectorXd p1 = VectorXd( n_x_ );
    if( fabs( psiD ) <= 1.0e-3 ) // check for very small values instead for == 0.0
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
    

    x_k += p1 + p2;
    // write predicted sigma points into right column
    Xsig_pred_.col(i) = x_k;
  }
  DEBUG(9, std::cout << __LINE__ << " leaving " << __FUNCTION__ << std::endl );
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::PredictMeanAndCovariance()
{
  DEBUG(9, std::cout << __LINE__ << " entering " << __FUNCTION__ << std::endl );
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
    t(3) = NormalizeAngle( t(3) );

    P_ += weights_(i) * ( t * t.transpose() );
  }
  DEBUG(9, std::cout << __LINE__ << " leaving " << __FUNCTION__ << std::endl );
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void UKF::PredictRadarMeasurement(MatrixXd &Zsig_out, VectorXd& z_pred, MatrixXd& S_out)
{
  DEBUG(9, std::cout << __LINE__ << " entering " << __FUNCTION__ << std::endl );
  // create matrix for sigma points in measurement space
  Zsig_out = MatrixXd( n_z_radar_, 2 * n_aug_ + 1 );

  // mean predicted measurement
  z_pred = VectorXd( n_z_radar_ );
  
  // measurement covariance matrix S
  S_out = MatrixXd( n_z_radar_, n_z_radar_ );

  // transform sigma points into measurement space
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
      double px   = Xsig_pred_( 0, i );
      double py   = Xsig_pred_( 1, i );
      double nu   = Xsig_pred_( 2, i );
      double psi  = Xsig_pred_( 3, i );
      //double psid = Xsig_pred_( 4, i );

      double lx = sqrt( px * px + py * py );
      if( lx > 1.0e-4 )
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
  S_out.fill( 0.0 );
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
      VectorXd t( n_z_radar_ );
      t = Zsig_out.col(i) - z_pred;
      t(1) = NormalizeAngle( t(1) );

      S_out += weights_( i ) * ( t * t.transpose().eval() );
  }

  S_out( 0, 0 ) += std_radr_ * std_radr_;
  S_out( 1, 1 ) += std_radphi_ * std_radphi_;
  S_out( 2, 2 ) += std_radrd_ * std_radrd_;

  DEBUG(9, std::cout << __LINE__ << " leaving " << __FUNCTION__ << std::endl );
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// Update
void UKF::UpdateStateRadar(VectorXd& z_in, MatrixXd &Zsig_in, VectorXd& z_pred, MatrixXd& S_in)
{
  DEBUG(9, std::cout << __LINE__ << " entering " << __FUNCTION__ << std::endl );
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd( n_x_, n_z_radar_ );

  // calculate cross correlation matrix
  Tc.fill( 0.0 );
  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
    VectorXd x_diff( n_x_ );
    VectorXd z_diff( n_z_radar_ );
    x_diff = Xsig_pred_.col( i ) - x_;
    z_diff = Zsig_in.col( i ) - z_pred;

    x_diff(3) = NormalizeAngle( x_diff(3) );
    z_diff(1) = NormalizeAngle( z_diff(1) );

    Tc += weights_(i) * x_diff * z_diff.transpose().eval();
  }

  // calculate Kalman gain K;
  MatrixXd K( n_x_, n_z_radar_ );
  K = Tc * S_in.inverse();

  // update state mean and covariance matrix
  VectorXd z_diff( n_z_radar_ );
  z_diff = z_in - z_pred;

  z_diff(1) = NormalizeAngle( z_diff(1) );

  x_ += K * z_diff;
  P_ -= K * (S_in * K.transpose());

  // ------------------------ NIS ----------------------------
  NIS_radar_ = z_diff.transpose() * S_in * z_diff;
  if( NIS_radar_ > 7.815 ) NIS_radar_050_++;
  if( NIS_radar_ > 6.251 ) NIS_radar_100_++;
  if( NIS_radar_ > 0.584 ) NIS_radar_900_++;
  if( NIS_radar_ > 0.352 ) NIS_radar_950_++;
  NIS_radar_total_++;

  DEBUG(9, std::cout << __LINE__ << " leaving " << __FUNCTION__ << std::endl );
  DEBUG(8, std::cout << __LINE__ << " NIS radar " << NIS_radar_ << std::endl );
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
