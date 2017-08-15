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
    // Initialise a rmse vector to return the values
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    
    // check to ensure estimation and ground truth vector is populated
    if((estimations.size() != 0) && (ground_truth.size() != 0))
    {
        for(int i = 0; i < estimations.size(); i++)
        {
            VectorXd residual = estimations[i] - ground_truth[i];
            residual = residual.array() * residual.array(); // element wise square
            rmse += residual;
        }
        
        //Calculate the mean
        rmse = rmse / estimations.size();
        
        //Calculate mean squre root
        rmse = rmse.array().sqrt(); // element wise square root
    }
    
    return rmse;
}
