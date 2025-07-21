// -------------------------------------------------------------------------------
// // Advanced Kalman Filtering and Sensor Fusion Course - Linear Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "sensors.h"
#include "utils.h"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01 / 180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0 / 180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.2;
// -------------------------------------------------- //

void KalmanFilter::predictionStep(const GyroMeasurement gyro, double dt) {
    if (!isInitialised())
        return;

    Vector4d &state = getState();
    const double cur_v = state(3);
    const double cur_psi = state(2);
    const double d_x = dt * cur_v * cos(cur_psi);
    const double d_y = dt * cur_v * sin(cur_psi);
    const double d_psi = dt * gyro.psi_dot;
    const double d_v = 0.0;

    state += (Eigen::Vector4d() << d_x, d_y, d_psi, d_v).finished();
    state(2) = wrapAngle(state(2));

    Matrix4d &cov = getCovariance();

    const auto F =
        (Eigen::Matrix4d() <<
         1, 0, -dt * cur_v * sin(cur_psi), dt * cos(cur_psi),
         0, 1, dt * cur_v * cos(cur_psi),  dt * sin(cur_psi),
         0, 0, 1,                          0,
         0, 0, 0,                          1)
        .finished();

    Eigen::Matrix<double, 4, 2> G;
    G << 0, 0,
         0, 0,
         0, dt,
         dt, 0;

    Matrix2d Q_w;
    Q_w << ACCEL_STD * ACCEL_STD, 0,
           0, GYRO_STD * GYRO_STD;

    Matrix4d Q = G * Q_w * G.transpose();

    cov.noalias() = F * cov * F.transpose() + Q;
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas, double dt) {
    constexpr double r_var = GPS_POS_STD * GPS_POS_STD;

    if (isInitialised()) {
        Vector4d &state = getState();
        Matrix4d &cov = getCovariance();

        static const auto H = []() {
            return (Eigen::Matrix<double, 2, 4>() <<
                     1, 0, 0, 0,
                     0, 1, 0, 0)
                   .finished();
        }();

        static const Matrix2d R = r_var * Matrix2d::Identity();

        const Vector2d z(meas.x, meas.y);
        const Vector2d innovation = z - H * state;
        const Matrix2d S = H * cov * H.transpose() + R;
        const Eigen::Matrix<double, 4, 2> K = cov * H.transpose() * S.inverse();

        state.noalias() += K * innovation;
        state(2) = wrapAngle(state(2));

        const auto cov_cpy = cov;
        cov.noalias() = (Matrix4d::Identity() - K * H) * cov_cpy;
    } else {
        if (m_first_loop) {
            m_first_loop = false;
            m_first_gps = meas;
            return;
        }

        if (accumulate_dt < 0.5f) {
            accumulate_dt += dt;
            return;
        }
        accumulate_dt += dt;

        Vector4d state;
        Matrix4d cov = Matrix4d::Zero();

        const double dx = meas.x - m_first_gps.x;
        const double dy = meas.y - m_first_gps.y;
        if (dx*dx + dy*dy < 2) {
            return;
        }
        const double init_v = sqrt(dx * dx + dy * dy) / accumulate_dt;
        const double init_psi = std::atan2(dy, dx);

        state << meas.x, meas.y, init_psi, init_v;
        cov.diagonal() << r_var, r_var, INIT_PSI_STD * INIT_PSI_STD, INIT_VEL_STD * INIT_VEL_STD;

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::handleLidarMeasurements(
    const std::vector<LidarMeasurement> &dataset, const BeaconMap &map) {
    if (!isInitialised())
        return;

    auto &state = getState();
    auto &cov = getCovariance();

    for (const auto &meas : dataset) {
        const auto beac = map.getBeaconWithId(meas.id);
        if (beac.isEmpty())
            continue;

        const auto dx = beac.x - state(0);
        const auto dy = beac.y - state(1);
        const auto d_sqr = dx * dx + dy * dy;
        if (d_sqr < 1e-6)
            continue;

        const double d = sqrt(d_sqr);

        const Eigen::Vector2d meas_vec(meas.theta, meas.range);
        const Eigen::Vector2d expected_measure(
            wrapAngle(-state(2) + atan2(dy, dx)),
            d
        );

        auto innovation = (meas_vec - expected_measure).eval();
        innovation(0) = wrapAngle(innovation(0));

        const Matrix2d R = (Eigen::Matrix2d() <<
            LIDAR_THETA_STD * LIDAR_THETA_STD, 0,
            0, LIDAR_RANGE_STD * LIDAR_RANGE_STD)
            .finished();

        const auto jacobian = (Eigen::Matrix<double, 2, 4>() <<
            dy / d_sqr, -dx / d_sqr, -1, 0,
            -dx / d,    -dy / d,     0, 0)
            .finished();

        const Matrix2d S = jacobian * cov * jacobian.transpose() + R;
        const auto K = cov * jacobian.transpose() * S.inverse();

        state.noalias() += K * innovation;
        state(2) = wrapAngle(state(2));
        cov.noalias() = (Matrix4d::Identity() - K * jacobian) * cov;
    }
}

[[nodiscard]] Matrix2d KalmanFilter::getVehicleStatePositionCovariance() const noexcept {
    Matrix2d pos_cov = Matrix2d::Zero();
    const auto &cov = getCovariance();
    if (isInitialised() && cov.size() != 0) {
        pos_cov << cov(0, 0), cov(0, 1),
                   cov(1, 0), cov(1, 1);
    }
    return pos_cov;
}

[[nodiscard]] VehicleState KalmanFilter::getVehicleState() const noexcept {
    VehicleState ret = {};
    if (isInitialised()) {
        const auto &state = getState();
        const double psi = state(2);
        const double V = state(3);
        ret = VehicleState(state[0], state[1], psi, V);
    }
    return ret;
}

void KalmanFilter::predictionStep(double dt) {}
void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas,
                                          const BeaconMap &map) {}

