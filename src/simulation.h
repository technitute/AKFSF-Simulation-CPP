#ifndef INCLUDE_AKFSFSIM_SIMULATION_H
#define INCLUDE_AKFSFSIM_SIMULATION_H

#include <memory>
#include <vector>

#include "kalmanfilter.h"
#include "display.h"
#include "car.h"
#include "beacons.h"
#include "sensors.h"


struct SimulationParams
{
    std::string profile_name;
    double time_step;
    double end_time;

    bool gps_enabled;
    double gps_update_rate;
    double gps_position_noise_std;
    double gps_error_probability;
    double gps_denied_x;
    double gps_denied_y;
    double gps_denied_range;

    bool lidar_enabled;
    bool lidar_id_enabled;
    double lidar_update_rate;
    double lidar_range_noise_std;
    double lidar_theta_noise_std;

    bool gyro_enabled;
    double gyro_update_rate;
    double gyro_noise_std;
    double gyro_bias;

    double car_initial_x;
    double car_initial_y;
    double car_initial_psi;
    double car_initial_velocity;

    std::vector<std::shared_ptr<MotionCommandBase>> car_commands;

    SimulationParams()
    :profile_name(""),
     time_step(0.1),end_time(120),
     gps_enabled(true), gps_update_rate(1.0), gps_position_noise_std(3), gps_error_probability(0.0),gps_denied_x(0.0),gps_denied_y(0.0),gps_denied_range(-1.0),
     lidar_enabled(false), lidar_id_enabled(true), lidar_update_rate(10.0),lidar_range_noise_std(3),lidar_theta_noise_std(0.02),
     gyro_enabled(true), gyro_update_rate(10.0),gyro_noise_std(0.001), gyro_bias(0.0),
     car_initial_x(0.0),car_initial_y(0.0),car_initial_psi(0.0),car_initial_velocity(5.0)
    {}
};


class Simulation
{
    public:

        Simulation();
        void reset();
        void reset(SimulationParams sim_params);
        void update();
        void render(Display& disp);
        void increaseTimeMultiplier();
        void decreaseTimeMultiplier();
        void setTimeMultiplier(unsigned int multiplier);
        void increaseZoom();
        void decreaseZoom();
        void togglePauseSimulation();
        bool isPaused();
        bool isRunning();

    private:

        SimulationParams m_sim_parameters;
        KalmanFilter m_kalman_filter;
        Car m_car;
        BeaconMap m_beacons;
        GyroSensor m_gyro_sensor;
        GPSSensor m_gps_sensor;
        LidarSensor m_lidar_sensor;

        bool m_is_paused;
        bool m_is_running;
        int  m_time_multiplier;
        double m_view_size;

        double m_time;
        double m_time_till_gyro_measurement;
        double m_time_till_gps_measurement;
        double m_time_till_lidar_measurement;

        std::vector<GPSMeasurement> m_gps_measurement_history;
        std::vector<LidarMeasurement> m_lidar_measurement_history;

        std::vector<Vector2> m_vehicle_position_history;
        std::vector<Vector2> m_filter_position_history;

        std::vector<double> m_filter_error_x_position_history;
        std::vector<double> m_filter_error_y_position_history;
        std::vector<double> m_filter_error_heading_history;
        std::vector<double> m_filter_error_velocity_history;

};


#endif  // INCLUDE_AKFSFSIM_SIMULATION_H
