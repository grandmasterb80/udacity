#include <iostream>
#include <vector>
#include "Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
  /**
   * Compute the Jacobian Matrix
   */

  // predicted state example
  // px = 1, py = 2, vx = 0.2, vy = 0.4
  VectorXd x_predicted(4);
  x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd Hj = CalculateJacobian(x_predicted);

  cout << "x:" << endl << x_predicted << endl;
  cout << "Hj:" << endl << Hj << endl;

  return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
  double ds = px * px + py * py;
  // check division by zero
  if( ds < 0.00001 )
    return Hj;
  double d = sqrt( ds );
  // compute the Jacobian matrix
  double d3 = pow( d, 3.0 );
  Hj << px / d, py / d, 0.0, 0.0,
        -py / ds, px / ds, 0.0, 0.0,
        py * ( vx * py - vy * px ) / d3, px * ( vy * px - vx * py ) / d3, px / d, py / d;

  return Hj;
}
