#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

/**
 * Calculate RMSE between accumulated state estimations and ground truth.
 *
 * Inputs are vectors of accumulated estimations and corresponding ground truth.
 * Returns a vector of RMSE values for [px, py, vx, vy].
 */
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // Check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if ( (estimations.size() != ground_truth.size()) ||
      (estimations.size() == 0) ) {
    cout << "Invalid estimation or ground_truth data size" << endl;
    return rmse;
  }
  
  // Accumulate squared error (residual)
  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    
    // Coefficient-wise multiplication to square the error
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  
  // Calculate the mean of squared error
  rmse = rmse / estimations.size();
  
  // Calculate the Root of Mean Squared Error
  rmse = rmse.array().sqrt();
  
  return rmse;
}
