#include "sensors.h"
#include "beacons.h"
#include "utils.h"

// GPS Sensor
GPSSensor::GPSSensor():m_rand_gen(std::mt19937()),m_noise_std(0.0),m_error_prob(0.0),m_gps_denied_x(0.0),m_gps_denied_y(0.0),m_gps_denied_range(-1.0){}
void GPSSensor::reset(){m_rand_gen = std::mt19937();}
void GPSSensor::setGPSNoiseStd(double std){m_noise_std = std;}
void GPSSensor::setGPSErrorProb(double prob){m_error_prob = prob;}
void GPSSensor::setGPSDeniedZone(double x, double y, double r){m_gps_denied_x = x; m_gps_denied_y = y; m_gps_denied_range = r;}
GPSMeasurement GPSSensor::generateGPSMeasurement(double sensor_x, double sensor_y)
{
    GPSMeasurement meas;
    std::normal_distribution<double> gps_pos_dis(0.0,m_noise_std);
    std::uniform_real_distribution<double> gps_error_dis(0.0,1.0);
    meas.x = sensor_x + gps_pos_dis(m_rand_gen);
    meas.y = sensor_y + gps_pos_dis(m_rand_gen);
    if (gps_error_dis(m_rand_gen) < m_error_prob) {meas.x = 0;meas.y = 0;}
    double delta_x = sensor_x-m_gps_denied_x;
    double delta_y = sensor_y-m_gps_denied_y;
    double range = sqrt(delta_x*delta_x + delta_y*delta_y);
    if (range < m_gps_denied_range){meas.x = 0;meas.y = 0;}
    return meas;
}

// Gyro Sensor
GyroSensor::GyroSensor():m_rand_gen(std::mt19937()),m_noise_std(0.0),m_bias(0.0){}
void GyroSensor::reset(){m_rand_gen = std::mt19937();}
void GyroSensor::setGyroNoiseStd(double std){m_noise_std = std;}
void GyroSensor::setGyroBias(double bias){m_bias = bias;}
GyroMeasurement GyroSensor::generateGyroMeasurement(double sensor_yaw_rate)
{
    GyroMeasurement meas;
    std::normal_distribution<double> gyro_dis(0.0,m_noise_std);
    meas.psi_dot = sensor_yaw_rate + m_bias + gyro_dis(m_rand_gen);
    return meas;
}

// Lidar Sensor
LidarSensor::LidarSensor():m_rand_gen(std::mt19937()),m_range_noise_std(0.0),m_theta_noise_std(0.0),m_max_range(90.0),m_id_enabled(true){}
void LidarSensor::reset(){m_rand_gen = std::mt19937();}
void LidarSensor::setLidarNoiseStd(double range_std, double theta_std){m_range_noise_std = range_std;m_theta_noise_std = theta_std;}
void LidarSensor::setLidarMaxRange(double range){m_max_range = range;}
void LidarSensor::setLidarDAEnabled(bool id_enabled){m_id_enabled = id_enabled;}
std::vector<LidarMeasurement> LidarSensor::generateLidarMeasurements(double sensor_x, double sensor_y, double sensor_yaw, const BeaconMap& map)
{
    std::vector<LidarMeasurement> meas;
    std::normal_distribution<double> lidar_theta_dis(0.0,m_theta_noise_std);
    std::normal_distribution<double> lidar_range_dis(0.0,m_range_noise_std);
    for (const auto& beacon : map.getBeacons())
    {
        double delta_x = beacon.x - sensor_x;
        double delta_y = beacon.y - sensor_y;
        double theta = wrapAngle(atan2(delta_y,delta_x) - sensor_yaw);
        double beacon_range = std::sqrt(delta_x*delta_x + delta_y*delta_y);
        if (beacon_range < m_max_range)
        {
            LidarMeasurement beacon_meas;
            beacon_meas.range = std::abs(beacon_range + lidar_range_dis(m_rand_gen));
            beacon_meas.theta = wrapAngle(theta + lidar_theta_dis(m_rand_gen));
            beacon_meas.id = (m_id_enabled ? beacon.id : -1);
            meas.push_back(beacon_meas);
        }
    }
    return meas;
}

