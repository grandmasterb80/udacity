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
  for( int i = 0; i < Xsig_pred.cols(); i++)
  {
    VectorXd x_k = Xsig_aug.block( 0, i, n_x, 1 );
    double px            = x_k[ 0 ];
    double py            = x_k[ 1 ];
    double v             = x_k[ 2 ];
    double psi           = x_k[ 3 ];
    double psiD          = x_k[ 4 ];
    double upsilon_a     = Xsig_aug( 5, i );
    double upsilon_psiDD = Xsig_aug( 6, i );

    // avoid division by zero
    VectorXd p1 = VectorXd( n_x );
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
    VectorXd p2 = VectorXd( n_x );
    p2 << 0.5 * delta_t * delta_t * cos( psi ) * upsilon_a,
          0.5 * delta_t * delta_t * sin( psi ) * upsilon_a,
          delta_t * upsilon_a,
          0.5 * delta_t * delta_t * upsilon_psiDD,
          delta_t * upsilon_psiDD;
    

    VectorXd x_kp1 = x_k + p1 + p2;
    //std::cout << "x_kp1 = " << std::endl << x_kp1 << std::endl;

    // write predicted sigma points into right column
    Xsig_pred.col(i) = Xsig_aug.block( 0, i, n_x, 1 ) + p1 + p2;
  }

  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;
  
  MatrixXd sollwert = MatrixXd( n_x, 2 * n_aug + 1);
  sollwert <<
    5.93553,  6.06251,   5.92217,   5.9415,    5.92361,   5.93516,   5.93705,  5.93553,   5.80832,   5.94481,   5.92935,   5.94553,   5.93589,   5.93401,  5.93553,
    1.48939,  1.44673,   1.66484,   1.49719,   1.508,     1.49001,   1.49022,  1.48939,   1.5308,    1.31287,   1.48182,   1.46967,   1.48876,   1.48855,  1.48939,
    2.2049,   2.28414,   2.24557,   2.29582,   2.2049,    2.2049,    2.23954,  2.2049,    2.12566,   2.16423,   2.11398,   2.2049,    2.2049,    2.17026,  2.2049,
    0.53678,  0.473387,  0.678098,  0.554557,  0.643644,  0.543372,  0.53678,  0.538512,  0.600173,  0.395462,  0.519003,  0.429916,  0.530188,  0.53678,  0.535048,
    0.3528,   0.299973,  0.462123,  0.376339,  0.48417,   0.418721,  0.3528,   0.387441,  0.405627,  0.243477,  0.329261,  0.22143,   0.286879,  0.3528,   0.318159;
  std::cout << "Xsig_soll = " << std::endl << sollwert << std::endl;
  std::cout << "Xsig_soll = " << std::endl << Xsig_pred - sollwert << std::endl;


  // write result
  *Xsig_out = Xsig_pred;
}

/*
Expected output:
  5.93553  6.06251   5.92217   5.9415    5.92361   5.93516   5.93705  5.93553   5.80832   5.94481   5.92935   5.94553   5.93589   5.93401  5.93553
  1.48939  1.44673   1.66484   1.49719   1.508     1.49001   1.49022  1.48939   1.5308    1.31287   1.48182   1.46967   1.48876   1.48855  1.48939
  2.2049   2.28414   2.24557   2.29582   2.2049    2.2049    2.23954  2.2049    2.12566   2.16423   2.11398   2.2049    2.2049    2.17026  2.2049
  0.53678  0.473387  0.678098  0.554557  0.643644  0.543372  0.53678  0.538512  0.600173  0.395462  0.519003  0.429916  0.530188  0.53678  0.535048
  0.3528   0.299973  0.462123  0.376339  0.48417   0.418721  0.3528   0.387441  0.405627  0.243477  0.329261  0.22143   0.286879  0.3528   0.318159
*/
/*
  5.7441   5.85768   5.7441    5.7441    5.7441    5.7441    5.7441   5.7441  5.63052   5.7441    5.7441    5.7441    5.7441    5.7441  5.96071
    1.38   1.34566   1.52806   1.38      1.38      1.38      1.38     1.38    1.41434   1.23194   1.38      1.38      1.38      1.38    1.50319
  2.2049   2.28414   2.24557   2.29582   2.2049    2.2049    2.2049   2.2049  2.12566   2.16423   2.11398   2.2049    2.2049    2.2049  2.77931
  0.5015   0.44339   0.631886  0.516923  0.595227  0.5015    0.5015   0.5015  0.55961   0.371114  0.486077  0.407773  0.5015    0.5015  0.5655
  0.3528   0.299973  0.462123  0.376339  0.48417   0.418721  0.3528   0.3528  0.405627  0.243477  0.329261  0.22143   0.286879  0.3528  0.92721
*/
