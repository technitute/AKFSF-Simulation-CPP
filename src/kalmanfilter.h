#pragma once

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
        void reset(){m_initialised = false;}
        bool isInitialised() const {return m_initialised;}

        virtual VehicleState getVehicleState() = 0;
        virtual Matrix2d getVehicleStatePositionCovariance() = 0;

        virtual void predictionStep(double dt) = 0;
        virtual void predictionStep(GyroMeasurement gyro, double dt) = 0;
        virtual void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map) = 0;
        virtual void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map) = 0;
        virtual void handleGPSMeasurement(GPSMeasurement meas) = 0;

    protected:
    
        VectorXd getState() const {return m_state;}
        MatrixXd getCovariance()const {return m_covariance;}
        void setState(const VectorXd& state ) {m_state = state; m_initialised = true;}
        void setCovariance(const MatrixXd& cov ){m_covariance = cov;}

    private:
    
        bool m_initialised;
        VectorXd m_state;
        MatrixXd m_covariance;
};

class KalmanFilterLKF : public KalmanFilterBase 
{
    public:

        KalmanFilterLKF():KalmanFilterBase(){};
        ~KalmanFilterLKF(){};

        VehicleState getVehicleState();
        Matrix2d getVehicleStatePositionCovariance();

        void predictionStep(double dt);
        void predictionStep(GyroMeasurement gyro, double dt);
        void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map);
        void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map);;
        void handleGPSMeasurement(GPSMeasurement meas);
};

class KalmanFilterEKF : public KalmanFilterBase
{
    public:

        KalmanFilterEKF():KalmanFilterBase(){};
        ~KalmanFilterEKF(){};

        VehicleState getVehicleState();
        Matrix2d getVehicleStatePositionCovariance();

        void predictionStep(double dt);
        void predictionStep(GyroMeasurement gyro, double dt);
        void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map);
        void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map);;
        void handleGPSMeasurement(GPSMeasurement meas);
};

class KalmanFilterUKF : public KalmanFilterBase
{
        public:

        KalmanFilterUKF():KalmanFilterBase(){};
        ~KalmanFilterUKF(){};

        VehicleState getVehicleState();
        Matrix2d getVehicleStatePositionCovariance();

        void predictionStep(double dt);
        void predictionStep(GyroMeasurement gyro, double dt);
        void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map);
        void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map);;
        void handleGPSMeasurement(GPSMeasurement meas);
};