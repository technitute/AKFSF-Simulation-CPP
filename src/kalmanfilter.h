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

        KalmanFilterBase() : m_initialised(false),
                             m_init_position_valid(false),
                             m_init_heading_valid(false),
                             m_init_velocity_valid(false),
                             m_init_bias_valid(false)
        {}
        virtual ~KalmanFilterBase(){}
        void reset()
        {
            m_initialised = false; 
            m_init_position_valid = false;
            m_init_heading_valid = false;
            m_init_velocity_valid = false;
            m_init_bias_valid = false;
        }
        bool isInitialised() const {return m_initialised;}

    protected:
    
        VectorXd getState() const {return m_state;}
        MatrixXd getCovariance()const {return m_covariance;}
        void setState(const VectorXd& state ) {m_state = state; m_initialised = true;}
        void setCovariance(const MatrixXd& cov ){m_covariance = cov;}

    private:
        bool m_initialised;
        VectorXd m_state;
        MatrixXd m_covariance;
        
    protected:
        double m_init_position_x;
        double m_init_position_y;
        double m_init_heading;
        double m_init_velocity;
        double m_init_bias;
        bool m_init_position_valid;
        bool m_init_heading_valid;
        bool m_init_velocity_valid;
        bool m_init_bias_valid;
};

class KalmanFilter : public KalmanFilterBase
{
    public:

        VehicleState getVehicleState();
        Matrix2d getVehicleStatePositionCovariance();

        void predictionStep(double dt);
        void predictionStep(GyroMeasurement gyro, double dt);
        void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map);
        void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map);
        void handleGPSMeasurement(GPSMeasurement meas);

};

#endif  // INCLUDE_AKFSFSIM_KALMANFILTER_H