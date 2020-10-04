#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // Student BAUDISCH - Start
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  /**
   * Create the augmented sigma points by computing also the augmented state and covariance matrix.
   * The function will store the result in Xsig_aug_.
   */
  void GenerateAugmentedSigmaPoints();

  /**
   * Predicts the mean and covariance for the current augmented sigma points (Xsig_aug_). The
   * result will be stored in Xsig_pred_.
   * @param delta_t          The time difference between current state and the predicted state.
   */
  void SigmaPointPrediction(double delta_t);

  /**
   * Predicts the mean and covariances based on the class member Xsig_pred_. The update will
   * be written to the members x_ and P_.
   */
  void PredictMeanAndCovariance();

  /**
   * Predict the next step for a radar measurement.
   */
  void PredictRadarMeasurement(Eigen::MatrixXd &Zsig_out, Eigen::VectorXd& z_pred, Eigen::MatrixXd& S_out);

  /**
   * Update the state for a Radar measurement.
   */
  void UpdateStateRadar(Eigen::VectorXd& z_in, Eigen::MatrixXd &Zsig_in, Eigen::VectorXd& z_pred, Eigen::MatrixXd& S_in);

  /// Radar measurement dimension
  int n_z_radar_;
  /// Lidar measurement dimension
  int n_z_lidar_;

  /// Normalized Innovation Squared (NIS) for radar (see https://classroom.udacity.com/nanodegrees/nd313/parts/da5e72fc-972d-42ae-bb76-fca3d3b2db06/modules/c32eb575-0080-4a25-a087-98ab84cb3092/lessons/daf3dee8-7117-48e8-a27a-fc4769d2b954/concepts/f3ba9445-452d-4727-8209-b317d44ff1f1)
  double NIS_radar_;
  unsigned int NIS_radar_950_;
  unsigned int NIS_radar_900_;
  unsigned int NIS_radar_100_;
  unsigned int NIS_radar_050_;
  unsigned int NIS_radar_total_;

  /// Normalized Innovation Squared (NIS) for lidar (see https://classroom.udacity.com/nanodegrees/nd313/parts/da5e72fc-972d-42ae-bb76-fca3d3b2db06/modules/c32eb575-0080-4a25-a087-98ab84cb3092/lessons/daf3dee8-7117-48e8-a27a-fc4769d2b954/concepts/f3ba9445-452d-4727-8209-b317d44ff1f1)
  double NIS_lidar_;
  unsigned int NIS_lidar_950_;
  unsigned int NIS_lidar_900_;
  unsigned int NIS_lidar_100_;
  unsigned int NIS_lidar_050_;
  unsigned int NIS_lidar_total_;

  /// ID of this filter - to limit debugging to specific objects
  int FID;

  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  /// Student BAUDISCH - End
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // augmented sigma covariance matrix
  Eigen::MatrixXd Xsig_aug_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;
};

#endif  // UKF_H
