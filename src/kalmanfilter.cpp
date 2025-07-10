// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Linear Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <vector>

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr bool INIT_ON_FIRST_PREDICTION = true;
constexpr double INIT_POS_STD = 1;
constexpr double INIT_VEL_STD = 10;
constexpr double ACCEL_STD = 0.1;
constexpr double GPS_POS_STD = 1.0;
// -------------------------------------------------- //

void KalmanFilter::predictionStep(double dt)
{
    constexpr double yaw = 45 * M_PI / 180.0;
    constexpr double cos_yaw = std::cos(yaw);
    constexpr double sin_yaw = std::sin(yaw);

    if (!isInitialised() && INIT_ON_FIRST_PREDICTION)
    {
        Vector4d state;
        Matrix4d cov = Matrix4d::Zero();

        state << 0, 0, 5.0 * cos_yaw, 5.0 * sin_yaw;

        cov.diagonal() << INIT_POS_STD * INIT_POS_STD,
                          INIT_POS_STD * INIT_POS_STD,
                          INIT_VEL_STD * INIT_VEL_STD,
                          INIT_VEL_STD * INIT_VEL_STD;

        setState(state);
        setCovariance(cov);
    }

    if (isInitialised())
    {
        Vector4d& state = getState();
        Matrix4d& cov = getCovariance();

        // State transition matrix F
        Matrix4d F = Matrix4d::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;

        // Predict new state
        state = F * state;

        // Predict new covariance
        if (ACCEL_STD > 1e-8)
        {
            // Q = G * M * G^T
            Eigen::Matrix<double, 4, 2> G;
            G << 0.5 * dt * dt, 0,
                 0, 0.5 * dt * dt,
                 dt, 0,
                 0, dt;

            Matrix2d M;
            M << ACCEL_STD * ACCEL_STD * cos_yaw, 0,
                 0, ACCEL_STD * ACCEL_STD * sin_yaw;

            cov.noalias() = F * cov * F.transpose() + G * M * G.transpose();
        }
        else
        {
            // Avoid unnecessary computation
            cov.noalias() = F * cov * F.transpose();
        }
    }
}


void KalmanFilter::handleGPSMeasurement(const GPSMeasurement& meas)
{
    constexpr double r_var = GPS_POS_STD * GPS_POS_STD;

    if (isInitialised())
    {
        Vector4d& state = getState();
        Matrix4d& cov = getCovariance();

        // Observation matrix: H maps [x, y, vx, vy] → [x, y]
        static const auto H = [](){
            return (Eigen::Matrix<double, 2, 4>() <<
            1, 0, 0, 0,
            0, 1, 0, 0).finished();
        }();

        static const Matrix2d R = r_var * Matrix2d::Identity();

        const Vector2d z(meas.x, meas.y);
        const Vector2d innovation = z - H * state;

        // Innovation covariance
        const Matrix2d S = H * cov * H.transpose() + R;

        // Kalman Gain (K = PHᵗS⁻¹)
        const Eigen::Matrix<double, 4, 2> K = cov * H.transpose() * S.inverse();

        // Update step
        state.noalias() += K * innovation;
        const auto cov_cpy = cov;
        cov.noalias() = (Matrix4d::Identity() - K * H).eval() * cov_cpy;

    }
    else
    {
        Vector4d state;
        Matrix4d cov = Matrix4d::Zero();

        state << meas.x, meas.y, 0, 0;
        cov.diagonal() << r_var, r_var, INIT_VEL_STD * INIT_VEL_STD, INIT_VEL_STD * INIT_VEL_STD;

        setState(state);
        setCovariance(cov);
    }
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    const auto& cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    VehicleState ret = {};
    if (isInitialised())
    {
        const auto& state = getState(); // STATE VECTOR [X,Y,VX,VY]
        const double psi = std::atan2(state[3],state[2]);
        const double V = std::sqrt(state[2]*state[2] + state[3]*state[3]);
        ret = VehicleState(state[0],state[1],psi,V);
    }
    return ret;
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt){predictionStep(dt);}
void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map){}
void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map){}

