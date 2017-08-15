#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
    
  // Initialisation to be set to false to start with
  is_initialized_ = false;
  
  // State Dimension
  n_x_ = 5;
  
  // Augumented State Dimension
  n_aug_ = 7;
    
  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // X Sigma Predicted
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    
  // Weights of Sigma Points
  weights_ = VectorXd(2 * n_aug_ + 1);
    
    
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
    
    // Switch between radar or laser measurement only if states changes
    if ((meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) ||
        (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_))
    {
        /***********************************************************************
         *  Initialization
         **********************************************************************/
        if (!is_initialized_)
        {
            // Initialise State Vector
            x_ << 1, 1, 1, 1, 0.1;
            
            // Initialise state Co-variance Matrix
            // can later be tuned to see the effects
            P_ << 0.15, 0, 0, 0, 0,
                  0, 0.15, 0, 0, 0,
                  0, 0, 1, 0, 0,
                  0, 0, 0, 1, 0,
                  0, 0, 0, 0, 1;
            
            // Initialise timestamp
            previous_timestamp_ = meas_package.timestamp_;
            
            // Initialise measurement values if Laser
            if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
            {
                x_(0) = meas_package.raw_measurements_(0);
                x_(1) = meas_package.raw_measurements_(1);
            }
            
            // Initialise measurement values if Radar
            if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
            {
                /**
                 Convert radar from polar to cartesian coordinates and initialize state.
                */
                float rho = meas_package.raw_measurements_(0);
                float phi = meas_package.raw_measurements_(1);
                
                // Even though rho_dot is available cannot be initialised to
                // state vector as the CTRV model, velocity is from the object's perspective
                x_(0) = rho * cos(phi);
                x_(1) = rho * sin(phi);
            }
            return;
        }
        
        /***********************************************************************
         *  Prediction
         **********************************************************************/
        /**
         Prediction and update are based on new elapsed time.
         - Time is measured in seconds.
        */
        
        float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
        previous_timestamp_ = meas_package.timestamp_;
        
        Prediction(dt);
        
        /***********************************************************************
         *  Update
         **********************************************************************/
        
        if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            UpdateLidar(meas_package);
        }
        
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            UpdateRadar(meas_package);
        }
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
    /***************************************************************************
     *  Generate Sigma Points
     **************************************************************************/
    
    // Create Sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
    
    //Calculate square root of P
    MatrixXd A = P_.llt().matrixL();
    
    // Fill fisrt column of Xsig with State vector
    Xsig.col(0) = x_;
    
    // set remaining columns of sigma points
    for (int i = 0; i < n_x_; i++)
    {
        Xsig.col(i+1) = x_ + (sqrt(lambda_ + n_x_) * A.col(i));
        Xsig.col(i+1+n_x_) = x_ - (sqrt(lambda_ + n_x_) * A.col(i));
    }
    
    /***************************************************************************
     *  Augument Sigma Points
     **************************************************************************/
    
    // Create Augumented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    
    // Create augumented co-variance Matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
    // Create Sigma Points Matrix Augumented
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    // Fill first 5 values of aug vector with x state and rest zero
    x_aug.head(n_x_) = x_;
    x_aug(5) = 0;
    x_aug(5) = 0;
    
    // add values to Augumented Co-variance Matrix
    P_aug.fill(0.0);
    
    // Fill top left corner of the P_aug with P
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(5,5) = std_a_ * std_a_;
    P_aug(6,6) = std_yawdd_ * std_yawdd_;
    
    /***************************************************************************
     *  Predict Sigma Points
     **************************************************************************/
    
    
    /***************************************************************************
     *  Convert predicted Sigma Points tp Mean and Covariance
     **************************************************************************/
    
    
    

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
