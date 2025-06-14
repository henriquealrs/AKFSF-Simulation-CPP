// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Linear Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr bool INIT_ON_FIRST_PREDICTION = true;
constexpr double INIT_POS_STD = 0;
constexpr double INIT_VEL_STD = 0;
constexpr double ACCEL_STD = 0.1;
constexpr double GPS_POS_STD = 3.0;
// -------------------------------------------------- //

void KalmanFilter::predictionStep(double dt)
{
    constexpr double yaw = 45 * M_PI / 180.0;
    constexpr auto cos_yaw = cos(yaw);
    constexpr auto sin_yaw = sin(yaw);

    if (!isInitialised() && INIT_ON_FIRST_PREDICTION)
    {
        // Implement the State Vector and Covariance Matrix Initialisation in the
        // section below if you want to initialise the filter WITHOUT waiting for
        // the first measurement to occur. Make sure you call the setState() /
        // setCovariance() functions once you have generated the initial conditions.
        // Hint: Assume the state vector has the form [X,Y,VX,VY].
        // Hint: You can use the constants: INIT_POS_STD, INIT_VEL_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE
        VectorXd state = Vector4d::Zero();
        MatrixXd cov =  Matrix4d::Zero();
        // cov.block(2, 2, 2, 2) = (5.0/3.0) * (5.0 / 3.0) * Matrix2d::Identity();

        // Assume the initial position is (X,Y) = (0,0) m
        state(0) = 0;
        state(1) = 0;

        // Assume the initial velocity is 5 m/s at 45 degrees (VX,VY) = (5*cos(45deg),5*sin(45deg)) m/s
        state(2) = 5 * cos_yaw;
        state(3) = 5 * sin_yaw;

        setState(state);
        setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }

    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Prediction Step for the system in the
        // section below.
        // Hint: You can use the constants: ACCEL_STD
        // ----------------------------------------------------------------------- //
        Matrix4d state_update = Eigen::Matrix4d::Identity();
        state_update(0, 2) = dt;
        state_update(1, 3) = dt;

        Matrix2d process_model_noise;
        process_model_noise << ACCEL_STD * ACCEL_STD* cos_yaw, 0,
                               0, ACCEL_STD * ACCEL_STD* sin_yaw;
        MatrixXd process_noise_transform(4, 2);
        process_noise_transform <<
            .5 * dt*dt,  0,
            0,  .5*dt*dt,
            1,  0,
            0, 1;
        // std::cout << "Process model: " << state_update << "\n";

        state = state_update * state;
        cov = state_update * cov * state_update.transpose() + process_noise_transform * process_model_noise * process_noise_transform.transpose();

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the GPS Measurements in the
        // section below.
        // Hint: Assume that the GPS sensor has a 3m (1 sigma) position uncertainty.
        // Hint: You can use the constants: GPS_POS_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE


        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
    else
    {
        // Implement the State Vector and Covariance Matrix Initialisation in the
        // section below. Make sure you call the setState/setCovariance functions
        // once you have generated the initial conditions.
        // Hint: Assume the state vector has the form [X,Y,VX,VY].
        // Hint: You can use the constants: GPS_POS_STD, INIT_VEL_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE
        VectorXd state = Vector4d::Zero();
        MatrixXd cov = Matrix4d::Zero();



        setState(state);
        setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,VX,VY]
        double psi = std::atan2(state[3],state[2]);
        double V = std::sqrt(state[2]*state[2] + state[3]*state[3]);
        return VehicleState(state[0],state[1],psi,V);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt){predictionStep(dt);}
void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map){}
void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map){}

