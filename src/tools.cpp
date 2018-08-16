#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse = VectorXd::Zero(4);
  if (estimations.size()==0 || estimations.size()!=ground_truth.size()){
    cout<< "Invalid data or sizes of estimations and ground truth unequal" << endl;
    return rmse;
  }

  for (double i=0;i<estimations.size();++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
    }

    rmse /= (estimations.size()+1e-5);
    rmse = rmse.array().sqrt();
    return rmse;
}
