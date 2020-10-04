/**
 *  - first measurement - the filter will receive initial measurements of the bicycle's position relative to
 *    the car. These measurements will come from a radar or lidar sensor.
 *  - initialize state and covariance matrices - the filter will initialize the bicycle's position based on
 *    the first measurement.
 *  - then the car will receive another sensor measurement after a time period Δt\Delta{t}Δt.
 *  - predict - the algorithm will predict where the bicycle will be after time Δt\Delta{t}Δt. One basic way
 *    to predict the bicycle location after Δt\Delta{t}Δt is to assume the bicycle's velocity is constant; thus
 *    the bicycle will have moved velocity * Δt\Delta{t}Δt. In the extended Kalman filter lesson, we will assume
 *    the velocity is constant.
 *  - update - the filter compares the "predicted" location with what the sensor measurement says. The predicted
 *    location and the measured location are combined to give an updated location. The Kalman filter will put
 *    more weight on either the predicted location or the measured location depending on the uncertainty of
 *    each value.
 *  - then the car will receive another sensor measurement after a time period Δt\Delta{t}Δt. The algorithm
 *    then does another predict and update step.
 * 
 * 
 * Definition of Variables
 *  - x is the mean state vector. For an extended Kalman filter, the mean state vector contains information
 *    about the object's position and velocity that you are tracking. It is called the "mean" state vector
 *    because position and velocity are represented by a gaussian distribution with mean xxx.
 *  - P is the state covariance matrix, which contains information about the uncertainty of the object's
 *    position and velocity. You can think of it as containing standard deviations.
 *  - k represents time steps. So xkx_kxk​ refers to the object's position and velocity vector at time k.
 *  - The notation k+1∣kk+1|kk+1∣k refers to the prediction step. At time k+1k+1k+1, you receive a sensor
 *    measurement. Before taking into account the sensor measurement to update your belief about the object's
 *    position and velocity, you predict where you think the object will be at time k+1k+1k+1. You can predict
 *    the position of the object at k+1k+1k+1 based on its position and velocity at time kkk.
 *    Hence x_{k+1|k}​ means that you have predicted where the object will be at k+1k+1k+1 but have
 *    not yet taken the sensor measurement into account.
 *  - x_{k+1}​ means that you have now predicted where the object will be at time k+1k+1k+1 and then used the
 *    sensor measurement to update the object's position and velocity.
 */

/** 
 * Write a function 'filter()' that implements a multi-
 *   dimensional Kalman Filter for the example given
 */

#include <iostream>
#include <vector>
#include "Dense"

using std::cout;
using std::endl;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// Kalman Filter variables
VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);


int main() {
  /**
   * Code used as example to work with Eigen matrices
   */
  // design the KF with 1D motion
  //     rows x cols
  // x:  1 x 2
  // P:  2 x 2
  // u:  1 x 2
  // F:  2 x 2
  // H:  1 x 2
  // R:  1 x 1
  x = VectorXd(2);
  x << 0, 0;

  P = MatrixXd(2, 2);
  P << 1000, 0, 0, 1000;

  u = VectorXd(2);
  u << 0, 0;

  F = MatrixXd(2, 2);
  F << 1, 1, 0, 1;

  H = MatrixXd(1, 2);
  H << 1, 0;

  R = MatrixXd(1, 1);
  R << 1;

  I = MatrixXd::Identity(2, 2);

  Q = MatrixXd(2, 2);
  Q << 0, 0, 0, 0;

  // create a list of measurements
  VectorXd single_meas(1);
  single_meas << 1;
  measurements.push_back(single_meas);
  single_meas << 2;
  measurements.push_back(single_meas);
  single_meas << 3;
  measurements.push_back(single_meas);

  // call Kalman filter algorithm
  filter(x, P);

  return 0;
}


void filter(VectorXd &x, MatrixXd &P) {

  for (unsigned int n = 0; n < measurements.size(); ++n) {

    VectorXd z = measurements[n];
    // TODO: YOUR CODE HERE
    // -------------------------------------------------------
    double dt = 1.0; // delta time
    // KF Measurement update step
    F(0,1) = dt;
    VectorXd y = z.transpose() - H * x;

    MatrixXd S = H * P * H.transpose() + R;
    MatrixXd K = P * H.transpose() * S.inverse();
    // new state
    x = x + K * y;
    P = (I - K * H) * P;

    // -------------------------------------------------------
    // KF Prediction step
    // Put dt into the state transition matrix.
    // F(0,1) = dt;
    x = F * x;
    P = F * P * F.transpose();
		
    cout << "x=" << endl <<  x << endl;
    cout << "P=" << endl <<  P << endl;
  }
}
