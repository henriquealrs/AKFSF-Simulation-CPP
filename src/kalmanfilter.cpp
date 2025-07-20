// -------------------------------------------------------------------------------
// // Advanced Kalman Filtering and Sensor Fusion Course - Linear Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
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
constexpr double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //

void KalmanFilter::predictionStep(const GyroMeasurement gyro, double dt) {
  if (isInitialised()) {
    Vector4d &state = getState();
    const double cur_v = state(3);
    const double cur_psi = state(2);
    const double d_long = dt * cur_v;
    const double d_x = dt * cur_v * cos(cur_psi);
    const double d_y = dt * cur_v * sin(cur_psi);
    const double d_psi = dt * gyro.psi_dot;
    const double d_v = 0.0; // Under the model assumption that accel is a random
                            // distrubotion acc = N(0, ACCEL_STD ^2);

    state += (Eigen::Vector4d() << d_x, d_y, d_psi, d_v).finished();
    state(2) = wrapAngle(state(2));

    Matrix4d &cov = getCovariance();

    const auto F =
        (Eigen::Matrix4d() << 1, 0, -dt * cur_v * sin(cur_psi),
         dt * cos(cur_psi), 0, 1, dt * cur_v * cos(cur_psi), dt * sin(cur_psi),
         0, 0, 1, 0, 0, 0, 0, 1)
            .finished(); // Jacobian of process applied to state in t-1
    Eigen::Matrix<double, 4, 2> G;
    G << 0, 0, 0, 0, 0, dt, dt, 0;

    // Covariance of the noise sources
    Matrix2d Q_w;
    Q_w << ACCEL_STD * ACCEL_STD, 0, 0, GYRO_STD * GYRO_STD;

    Matrix4d Q = G * Q_w * G.transpose();

    cov.noalias() = F * cov * F.transpose() + Q;
  }
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas, double dt) {
  constexpr double r_var = GPS_POS_STD * GPS_POS_STD;

  if (isInitialised()) {
    Vector4d &state = getState();
    Matrix4d &cov = getCovariance();

    // Observation matrix: H maps [x, y, vx, vy] → [x, y]
    static const auto H = []() {
      return (Eigen::Matrix<double, 2, 4>() << 1, 0, 0, 0, 0, 1, 0, 0)
          .finished();
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

    } else {
        static auto first_capture = meas;
        if (m_first_loop) {
            Vector4d state;
            Matrix4d cov = Matrix4d::Zero();

            const double dx = meas.x - first_capture.x;
            const double dy = meas.y - first_capture.y;
            const double init_v = sqrt(dx * dx + dy * dy) / dt;
            const double init_psi = std::atan2(dy, dx);
            state << meas.x, meas.y, init_v, init_psi;
            cov.diagonal() << r_var, r_var, INIT_PSI_STD * INIT_PSI_STD,
                INIT_VEL_STD * INIT_VEL_STD;

            setState(state);
            setCovariance(cov);
        }
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
        if (beac.isEmpty()) {
            continue;
        }

        const auto state_x = state(0);
        const auto state_y = state(1);
        [[maybe_unused]] const auto state_psi = state(2);
        [[maybe_unused]] const auto state_v = state(3);

        const auto meas_vec =
            (Eigen::Vector2d() << meas.theta, meas.range * meas.range)
            .finished();

        const Eigen::Vector2d expected_measure =
            (Eigen::Vector2d() <<
                (-state_psi + atan2(beac.y - state_y, beac.x - state_x)),
                (beac.x - state_x) * (beac.x - state_x) + (beac.y - state_y) * (beac.y - state_y))
            .finished();

        const auto innovation = meas_vec - expected_measure;
        const auto R =
            (Eigen::Matrix2d() <<
            0, LIDAR_THETA_STD * LIDAR_THETA_STD,
            LIDAR_RANGE_STD * LIDAR_RANGE_STD, 0)
            .finished();

        const auto dy = (beac.y - state_y) * (state_y - beac.y);
        const auto dx = (beac.x - state_x) * (state_x - beac.x);
        const auto d_sqr = dx * dx + dy * dy;
        const auto d = sqrt(d_sqr);
        const auto jacobian = (
            Eigen::Matrix<double, 2, 4>() <<
            dy / d_sqr, /**/ -dx / d_sqr, /**/ /**/ 1, /**/ 0,
            -dx / d,    /**/ -dy / d,     /**/ /**/ 0, /**/ 0
        ).finished();

        const auto innovation_covariance = jacobian * cov * jacobian.transpose() + R;
        std::cout << "R = \n" << R << "------------\n";
        std::cout << "S = \n" << innovation_covariance << "------------\n";

        const auto kalman_gain = cov  * jacobian.transpose() * innovation_covariance.inverse();
        state += kalman_gain * innovation;
        cov = (Eigen::Matrix4d::Identity() - kalman_gain * jacobian) * cov;
    }
}

[[nodiscard]] Matrix2d
KalmanFilter::getVehicleStatePositionCovariance() const noexcept {
  Matrix2d pos_cov = Matrix2d::Zero();
  const auto &cov = getCovariance();
  if (isInitialised() && cov.size() != 0) {
    pos_cov << cov(0, 0), cov(0, 1), cov(1, 0), cov(1, 1);
  }
  return pos_cov;
}

[[nodiscard]] VehicleState KalmanFilter::getVehicleState() const noexcept {
  VehicleState ret = {};
  if (isInitialised()) {
    const auto &state = getState(); // STATE VECTOR [X,Y,VX,VY]
    const double psi = state(2);
    const double V = state(3);
    ret = VehicleState(state[0], state[1], psi, V);
  }
  return ret;
}

void KalmanFilter::predictionStep(double dt) {}
void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas,
                                          const BeaconMap &map) {}
