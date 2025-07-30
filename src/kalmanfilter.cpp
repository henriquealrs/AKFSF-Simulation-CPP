// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Unscented Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Cholesky/LLT.h>
#include <Eigen/Dense>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Eigenvalues/EigenSolver.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>


using Vector6d = Eigen::Vector<double, 6>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 0.05;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double INIT_VEL_STD = 2;
constexpr double INIT_PSI_STD = 5.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //
// TUNABLE UKF PARAMETERS
constexpr double ALPHA = 1e-3;
constexpr double BETA  = 2.0;
constexpr double KAPPA = 0.0;

// ----------------------------------------------------------------------- //
// USEFUL HELPER FUNCTIONS
namespace {
inline void normaliseState(Eigen::Vector4d& state)
{
    state(2) = wrapAngle(state(2));
}

inline void normaliseLidarMeasurement(Eigen::Vector4d& meas)
{
    meas(1) = wrapAngle(meas(1));
}

std::vector<Vector6d> generateSigmaPoints(const Vector6d& state, const Matrix6d& cov)
{
    const int n = state.size();
    const double lambda = ALPHA*ALPHA*(n+KAPPA) - n;
    const double scale  = std::sqrt(n + lambda);

    std::vector<Vector6d> sigmaPoint;
    sigmaPoint.reserve(n * 2 + 1);
    sigmaPoint.push_back(state);

    const auto sqrt_cov = Matrix6d(cov.llt().matrixL());

    for(int i = 0; i < n; ++i)
    {
        sigmaPoint.push_back( state + scale * sqrt_cov.col(i));
        sigmaPoint.push_back( state - scale * sqrt_cov.col(i));
    }

    return sigmaPoint;
}

std::vector<double> generateSigmaWeights(int n)
{
    const double lambda = ALPHA*ALPHA*(n+KAPPA) - n;
    std::vector<double> W(2*n+1);
    W[0] = lambda / (n + lambda);
    for(int i = 1; i < 2*n+1; ++i)
        W[i] = 1.0 / (2.0 * (n + lambda));
    return W;
}
std::vector<double> weights;

}


namespace {
Vector4d vehicleProcessModel(const Vector6d& aug_state, const double psi_dot, const double dt)
{
    const auto v = aug_state(3);
    const auto psi = aug_state(2);
    Vector4d new_state = aug_state.block<4, 1>(0, 0);
    new_state(0) += v * dt * cos(psi);
    new_state(1) += v * dt * sin(psi);
    new_state(2) += dt * (psi_dot + aug_state(4));
    new_state(3) += dt * aug_state(5);

    return new_state;
}
}
// ----------------------------------------------------------------------- //

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt)
{
    if (!isInitialised()) return;

    auto& state = getState();         // [px, py, psi, v]
    auto& cov = getCovariance();      // 4x4 covariance

    // --- Augment state and covariance ---
    Vector6d aug_state;
    aug_state << state(0), state(1), state(2), state(3), 0, 0;

    Matrix6d aug_cov = Matrix6d::Zero();
    aug_cov.block<4,4>(0,0) = cov;
    aug_cov(4,4) = GYRO_STD * GYRO_STD;
    aug_cov(5,5) = ACCEL_STD * ACCEL_STD;

    // --- Sigma point generation ---
    const int n_aug = aug_state.size();
    double alpha = 1e-3;
    double kappa = 0;
    double beta = 2;  // optimal for Gaussian

    double lambda = alpha*alpha*(n_aug + kappa) - n_aug;

    // Cholesky for sigma points
    Matrix6d sqrt_cov = aug_cov.llt().matrixL();
    std::vector<Vector6d> sigma_points;
    sigma_points.reserve(2*n_aug + 1);

    sigma_points.push_back(aug_state);
    double scale = sqrt(n_aug + lambda);
    for (int i = 0; i < n_aug; ++i)
    {
        sigma_points.push_back(aug_state + scale * sqrt_cov.col(i));
        sigma_points.push_back(aug_state - scale * sqrt_cov.col(i));
    }

    // --- Weights ---
    std::vector<double> weights;
    weights.reserve(2*n_aug + 1);
    weights.push_back(lambda / (n_aug + lambda));
    for (int i = 0; i < 2*n_aug; ++i)
        weights.push_back(0.5 / (n_aug + lambda));

    // --- Predict sigma points ---
    std::vector<Eigen::Vector4d> predicted_sigma(sigma_points.size());
    for (size_t i = 0; i < sigma_points.size(); ++i)
    {
        predicted_sigma[i] = vehicleProcessModel(sigma_points[i], gyro.psi_dot, dt);
    }

    // --- Predicted mean ---
    Eigen::Vector4d x_pred = Eigen::Vector4d::Zero();
    for (size_t i = 0; i < predicted_sigma.size(); ++i)
        x_pred += weights[i] * predicted_sigma[i];

    normaliseState(x_pred); // keep yaw clean

    // --- Predicted covariance ---
    Matrix4d P_pred = Matrix4d::Zero();
    for (size_t i = 0; i < predicted_sigma.size(); ++i)
    {
        Eigen::Vector4d diff = predicted_sigma[i] - x_pred;
        normaliseState(diff);  // yaw normalization in diff
        P_pred += weights[i] * (diff * diff.transpose());
    }

    // --- Store results ---
    state = x_pred;
    cov = P_pred;
}

// void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt)
// {
//     if (isInitialised())
//     {
//         auto& state = getState();
//         auto& cov = getCovariance();
//
//         // Implement The Kalman Filter Prediction Step for the system in the
//         // section below.
//         // HINT: Assume the state vector has the form [PX, PY, PSI, V].
//         // HINT: Use the Gyroscope measurement as an input into the prediction step.
//         // HINT: You can use the constants: ACCEL_STD, GYRO_STD
//         // HINT: Use the normaliseState() function to always keep angle values within correct range.
//         // HINT: Do NOT normalise during sigma point calculation!
//         // ----------------------------------------------------------------------- //
//         // ENTER YOUR CODE HERE
//         const auto aug_state = (Eigen::Vector<double, 6>() << state(0), state(1), state(2), state(3), 0, 0).finished();
//         const auto aug_cov = [](const Matrix4d& cov) {
//             Eigen::Matrix<double, 6, 6> ret = Eigen::Matrix<double, 6, 6>::Zero();
//             ret.block<4, 4>(0, 0) = cov;
//
//             Eigen::Matrix2d noise_cov;
//             noise_cov << GYRO_STD * GYRO_STD, 0,
//                 0, ACCEL_STD * ACCEL_STD;
//             ret.block<2, 2>(4, 4) = noise_cov;
//
//             return ret;
//         }(cov);
//
//         const std::vector<Vector6d> sigma_states = generateSigmaPoints(aug_state, aug_cov);
//         const std::vector<double> weights = generateSigmaWeights(6);
//
//         auto final_state = Eigen::Vector4d::Zero().eval();
//         for(unsigned i = 0; i < sigma_states.size(); ++i)
//         {
//             const auto& aug_state = sigma_states[i];
//             const auto w = weights[i];
//             auto ns = w * vehicleProcessModel(aug_state, gyro.psi_dot, dt);
//             final_state += ns;
//         }
//
//         state = final_state;
//         // ----------------------------------------------------------------------- //
//     }
// }


namespace {
VectorXd lidarMeasurementModel(VectorXd aug_state, double beaconX, double beaconY)
{
    VectorXd z_hat = VectorXd::Zero(2);

    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE

    // ----------------------------------------------------------------------- //

    return z_hat;
}
}


void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the Lidar Measurements in the
        // section below.
        // HINT: Use the normaliseState() and normaliseLidarMeasurement() functions
        // to always keep angle values within correct range.
        // HINT: Do not normalise during sigma point calculation!
        // HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
        // HINT: The mapped-matched beacon position can be accessed by the variables
        // map_beacon.x and map_beacon.y
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built in Data Association Id
        if (meas.id != -1 && map_beacon.id != -1) // Check that we have a valid beacon match
        {


        }
        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas, double dt) {
    constexpr double r_var = GPS_POS_STD * GPS_POS_STD;

    if (isInitialised()) {
        Vector4d &state = getState();
        Matrix4d &cov = getCovariance();

        static const auto H = (Eigen::Matrix<double, 2, 4>() <<
                     1, 0, 0, 0,
                     0, 1, 0, 0)
                   .finished();

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

void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const auto& meas : dataset) {handleLidarMeasurement(meas, map);}
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance() const noexcept
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState() const noexcept
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,PSI,V,...]
        return VehicleState(state[0],state[1],state[2],state[3]);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(double dt){}
