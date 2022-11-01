#ifndef EKFSLAM_H
#define EKFSLAM_H
#include <vector>
#include "../include/sensor_info.h"
#include "../include/common.h"
#include "../include/Eigen/Dense"
#define INF 1000

class EKFSLAM {
 private:
  // Covariance Matrix for robot state variables
  Eigen::MatrixXd robotSigma;
  // Covariance Matrix for robot to landmarks
  Eigen::MatrixXd robMapSigma;
  // Covariances of landmark positions wrt to each other
  Eigen::MatrixXd mapSigma;
  // Full Covariance Matrix
  Eigen::MatrixXd Sigma;
  // Full State Vector
  Eigen::VectorXd mu;
  // Noise Matrix due to motion
  Eigen::MatrixXd R;
  // Noise Matrix due to sensors
  Eigen::MatrixXd Q;
  // Vector of observed landmarks
  vector<bool> observedLandmarks;

  // TODO: Complete Implementation. You can define your own helper/auxilary functions.
  //! You MUST have the following functions:

  // Prediction()
  // Description: Prediction step for the EKF based off an odometry model
  // Inputs:
  // control input for one time step

  // Correction()
  // Description: Correction step for EKF
  // Inputs:
  //  all observed landmarks from a laser scanner for said time step

};

#endif  // EKFSLAM_H
