#include <iostream>
#include <cmath>
#include "../include/objects/Robot.h"
#include "../include/tools.h"
#include "../include/objects/Robot.h"
#include "eigen3/Eigen/Dense"

using namespace Eigen;

// KalmanFilter class
class KalmanFilter
{
public:
    //Constructor
    KalmanFilter()
    {
        // Initialize state variables
        x = VectorXd(3);  // [x, y, theta]
        x << 0, 0, 0;

        // Initialize state covariance matrix
        P = MatrixXd(3, 3);
        P << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;

        // Initialize process noise covariance matrix
        Q = MatrixXd(3, 3);
        Q << 0.1, 0, 0,
             0, 0.1, 0,
             0, 0, 0.1;

        // Initialize measurement noise covariance matrix
        R = MatrixXd(2, 2);
        R << 0.1, 0,
             0, 0.1;

        //Measurement matrix
        H = MatrixXd(2,3);
        H << 1, 0, 0,
             0, 1, 0;
    }

    // Prediction step: predict the state estimate
    Vector3d predict(double dt, Vector2d vel)
    {
        double lin_vel = vel[0];
        double ang_vel = vel[1];
        
        // Update state transition matrix A
        A << 1, 0, -lin_vel * dt * sin(x(2)),
             0, 1, lin_vel * dt * cos(x(2)),
             0, 0, 1;

        // Update control input matrix B
        B << dt * cos(x(2)), 0,
             dt * sin(x(2)), 0,
             0, dt;

        // Update control input vector u
        u = vel;

        // Predict the state estimate
        x = A * x + B * u;

        // Update the state covariance matrix
        P = A * P * A.transpose() + Q;
    }

    // Update step: update the state estimate based on the measurement
    void update(const Vector2d& measurement)
    {

        // Calculate the measurement residual
        Vector2d y = measurement - H * x;

        // Calculate the residual covariance
        Matrix2d S = H * P * H.transpose() + R;

        // Calculate the Kalman gain
        MatrixXd K = P * H.transpose() * S.inverse();

        // Update the state estimate
        x = x + K * y;

        // Update the state covariance matrix
        P = (MatrixXd::Identity(3, 3) - K * H) * P;
    }

    // Get the current state estimate
    Vector3d getState()
    {
        return x;
    }

    // State variables
    Vector3d x;  // [x, y, theta]

    // State covariance matrix
    Matrix3d P;

    // Process noise covariance matrix
    Matrix3d Q;

    // Measurement noise covariance matrix
    Matrix2d R;

    // State transition matrix
    Matrix3d A;

    // Control input matrix
    MatrixXd B;

    // Control input vector
    Vector2d u;

    // Measurement matrix
    Matrix<double, 2, 3> H;
};




RobotPose kalmanEstimate(emc::OdometryData &odom, RobotPose Pose, Vector2d vel, Matrix3d &P, int prev_t)
{
    // Create a Kalman filter object
    KalmanFilter kf;

    kf.P = P;

    double sigma_x = 0.1, sigma_y = 0.1, sigma_a = 0.1;

    Vector3d predictState, state;

    state << Pose.x, 
             Pose.y,
             Pose.a;
    
    
    if(prev_t == -1) //First instance
    {
        //First guess as current state
        state << odom.x, odom.y, odom.a;
        
        kf.P << sigma_x, 0, 0,
             0, sigma_y, 0,
             0, 0, sigma_a;
        
        predictState = state;
        return predictState;
    }
    
    double dt;

    dt = odom.timestamp - prev_t;

    // Perform prediction and update steps
    predictState = kf.predict(dt, vel);

    kf.update(predictState);

    P = kf.P;

    predictState =  kf.getState();

    return predictState;

}
