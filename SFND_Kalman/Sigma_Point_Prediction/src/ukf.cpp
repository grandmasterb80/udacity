#include <iostream>
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
  Init();
}

UKF::~UKF() {

}

void UKF::Init() {

}


/**
 * Programming assignment functions: 
 */

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; // time diff in sec

  /**
   * Student part begin
   */

  // predict sigma points
  VectorXd x_k = Xsig_aug.block( 0, 0, n_x, 1 );
  double px            = x_k[ 0 ];
  double py            = x_k[ 1 ];
  double v             = x_k[ 2 ];
  double psi           = x_k[ 3 ];
  double psiD          = x_k[ 4 ];
  double upsilon_a     = Xsig_aug( 0, 5 );
  double upsilon_psiDD = Xsig_aug( 0, 6 );

  // avoid division by zero
  VectorXd p1 = VectorXd( n_x );
  if( fabs( psiD ) <= 1.0e-6 ) // check for very small values instead for == 0.0
  {
      p1 << v / psiD * ( sin( psi + psiD * delta_t ) - sin( psi ) ),
            v / psiD * (-cos( psi + psiD * delta_t ) + cos( psi ) ),
            0.0,
            psiD * deltaT,
            0.0;
  }
  else
  {
      p1 << v / psiD * ( sin( psi + psiD * delta_t ) - sin( psi ) ),
            v / psiD * (-cos( psi + psiD * delta_t ) + cos( psi ) ),
            0.0,
            psiD * deltaT,
            0.0;
  }
  VectorXd p2 = VectorXd( n_x );
  p2 << 0.5 * delta_t * delta_t * cos( psi ) *

  VectorXd x_kp1 = x_k + p1 + p2;

  // write predicted sigma points into right column
  Xsig_pred

  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  // write result
  *Xsig_out = Xsig_pred;
}
