#ifndef INCLUDE_AKFSFSIM_KALMANFILTER_H
#define INCLUDE_AKFSFSIM_KALMANFILTER_H

#include <vector>
#include <Eigen/Dense>

#include "car.h"
#include "sensors.h"
#include "beacons.h"

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector4d;

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix4d;

class KalmanFilterBase
{
    public:

        KalmanFilterBase():m_initialised(false){}
        virtual ~KalmanFilterBase(){}
        virtual void reset(){m_initialised = false;}
        bool isInitialised() const {return m_initialised;}

    protected:

        Vector4d& getState(){return m_state;}
        const Vector4d& getState() const noexcept{return m_state;}
        Matrix4d& getCovariance() {return m_covariance;}
        const Matrix4d& getCovariance() const noexcept {return m_covariance;}
        void setState(const Vector4d& state ) {m_state = state; m_initialised = true;}
        void setCovariance(const Matrix4d& cov ){m_covariance = cov;}

    private:
        bool m_initialised;
        Vector4d m_state;
        Matrix4d m_covariance;
};

class KalmanFilter : public KalmanFilterBase
{
    bool m_first_loop = true;
    GPSMeasurement m_first_gps = {};
    double accumulate_dt = 0.0;
public:
    virtual void reset() {
        m_first_loop = true;
        m_first_gps = {};
        accumulate_dt = 0.0;
        KalmanFilterBase::reset();
    }

    [[nodiscard]] VehicleState getVehicleState() const noexcept;
    [[nodiscard]] Matrix2d getVehicleStatePositionCovariance() const noexcept;

    void predictionStep(double dt);
    void predictionStep(GyroMeasurement gyro, double dt);
    void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map);
    void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map);
    void handleGPSMeasurement(GPSMeasurement meas, double dt);

};

#endif  // INCLUDE_AKFSFSIM_KALMANFILTER_H
