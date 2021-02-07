
#include "simulation.h"
#include "utils.h"


Simulation::Simulation()
: m_sim_parameters(SimulationParams()),
    m_is_paused(false),
    m_is_running(false),
    m_time_multiplier(1),
    m_view_size(100),
    m_time(0.0),
    m_time_till_gyro_measurement(0.0),
    m_time_till_gps_measurement(0.0),
    m_time_till_lidar_measurement(0.0)
{}

void Simulation::reset()
{
    // Reset Simulation
    m_time = 0.0;
    m_time_till_gyro_measurement = 0.0;
    m_time_till_gps_measurement = 0.0;
    m_time_till_lidar_measurement= 0.0;

    m_is_running = true;
    m_is_paused = false;
    
    m_kalman_filter.reset();

    m_gps_sensor.reset();
    m_gps_sensor.setGPSNoiseStd(m_sim_parameters.gps_position_noise_std);
    m_gps_sensor.setGPSErrorProb(m_sim_parameters.gps_error_probability);
    m_gps_sensor.setGPSDeniedZone(m_sim_parameters.gps_denied_x, m_sim_parameters.gps_denied_y, m_sim_parameters.gps_denied_range);

    m_gyro_sensor.reset();
    m_gyro_sensor.setGyroNoiseStd(m_sim_parameters.gyro_noise_std);
    m_gyro_sensor.setGyroBias(m_sim_parameters.gyro_bias);

    m_lidar_sensor.reset();
    m_lidar_sensor.setLidarNoiseStd(m_sim_parameters.lidar_range_noise_std, m_sim_parameters.lidar_theta_noise_std);
    m_lidar_sensor.setLidarDAEnabled(m_sim_parameters.lidar_id_enabled);

    m_car.reset(m_sim_parameters.car_initial_x, m_sim_parameters.car_initial_y,m_sim_parameters.car_initial_psi, m_sim_parameters.car_initial_velocity);
    
    for(auto& cmd : m_sim_parameters.car_commands){m_car.addVehicleCommand(cmd.get());}

    // Plotting Variables
    m_gps_measurement_history.clear();
    m_lidar_measurement_history.clear();
    m_vehicle_position_history.clear();
    m_filter_position_history.clear();

    // Stats Variables
    m_filter_error_x_position_history.clear();
    m_filter_error_y_position_history.clear();
    m_filter_error_heading_history.clear();
    m_filter_error_velocity_history.clear();

    std::cout << "Simulation: Reset" << std::endl;
}


void Simulation::update()
{
    if (m_is_running && !m_is_paused)
    {
        // Time Multiplier
        for (unsigned i = 0; i < m_time_multiplier; ++i)
        {
            // Check for End Time
            if(m_time >= m_sim_parameters.end_time)
            {
                m_is_running = false;
                std::cout << "Simulation: Reached End of Simulation Time (" << m_time << ")" << std::endl;
                return;
            }

            // Update Motion
            m_car.update(m_time, m_sim_parameters.time_step);
            m_vehicle_position_history.push_back(Vector2(m_car.getVehicleState().x,m_car.getVehicleState().y));

            // Gyro Measurement / Prediction Step
            if (m_sim_parameters.gyro_enabled)
            {
                if (m_time_till_gyro_measurement <= 0)
                {
                    GyroMeasurement meas = m_gyro_sensor.generateGyroMeasurement(m_car.getVehicleState().yaw_rate);
                    m_kalman_filter.predictionStep(meas, m_sim_parameters.time_step);
                    m_time_till_gyro_measurement += 1.0/m_sim_parameters.gyro_update_rate;
                }
                m_time_till_gyro_measurement -= m_sim_parameters.time_step;
            }

            // GPS Measurement
            if (m_sim_parameters.gps_enabled)
            {
                if (m_time_till_gps_measurement <= 0)
                {
                    GPSMeasurement gps_meas = m_gps_sensor.generateGPSMeasurement(m_car.getVehicleState().x,m_car.getVehicleState().y);
                    m_kalman_filter.handleGPSMeasurement(gps_meas);
                    m_gps_measurement_history.push_back(gps_meas);
                    m_time_till_gps_measurement += 1.0/m_sim_parameters.gps_update_rate;
                }
                m_time_till_gps_measurement -= m_sim_parameters.time_step;
            }

            // Lidar Measurement
            if (m_sim_parameters.lidar_enabled)
            {
                if (m_time_till_lidar_measurement <= 0)
                {
                    std::vector<LidarMeasurement> lidar_measurements = m_lidar_sensor.generateLidarMeasurements(m_car.getVehicleState().x,m_car.getVehicleState().y, m_car.getVehicleState().psi, m_beacons);
                    m_kalman_filter.handleLidarMeasurements(lidar_measurements, m_beacons);
                    m_lidar_measurement_history = lidar_measurements;
                    m_time_till_lidar_measurement += 1.0/m_sim_parameters.lidar_update_rate;
                }
                m_time_till_lidar_measurement -= m_sim_parameters.time_step;
            }

            // Save Filter History and Calculate Stats
            if (m_kalman_filter.isInitialised())
            {
                VehicleState vehicle_state = m_car.getVehicleState();
                VehicleState filter_state = m_kalman_filter.getVehicleState();
                m_filter_position_history.push_back(Vector2(filter_state.x, filter_state.y));
                m_filter_error_x_position_history.push_back(filter_state.x - vehicle_state.x);
                m_filter_error_y_position_history.push_back(filter_state.y - vehicle_state.y);
                m_filter_error_heading_history.push_back(wrapAngle(filter_state.psi - vehicle_state.psi));
                m_filter_error_velocity_history.push_back(filter_state.V - vehicle_state.V);
            }

            // Update Time
            m_time += m_sim_parameters.time_step;
        }
    }
}
        
void Simulation::render(Display& disp)
{
    std::vector<Vector2> marker_lines1 = {{0.5,0.5},{-0.5,-0.5}};
    std::vector<Vector2> marker_lines2 = {{0.5,-0.5},{-0.5,0.5}};

    disp.setView(m_view_size * disp.getScreenAspectRatio(),m_view_size, m_car.getVehicleState().x, m_car.getVehicleState().y);

    m_car.render(disp);
    m_beacons.render(disp);

    disp.setDrawColour(0,100,0);
    disp.drawLines(m_vehicle_position_history);

    disp.setDrawColour(100,0,0);
    disp.drawLines(m_filter_position_history);

    if (m_kalman_filter.isInitialised())
    {
        VehicleState filter_state = m_kalman_filter.getVehicleState();
        Eigen::Matrix2d cov = m_kalman_filter.getVehicleStatePositionCovariance();

        double x = filter_state.x;
        double y = filter_state.y;
        double sigma_xx = cov(0,0);
        double sigma_yy = cov(1,1);
        double sigma_xy = cov(0,1);

        std::vector<Vector2> marker_lines1_world = offsetPoints(marker_lines1, Vector2(x,y));
        std::vector<Vector2> marker_lines2_world = offsetPoints(marker_lines2, Vector2(x,y));
        disp.setDrawColour(255,0,0);
        disp.drawLines(marker_lines1_world);
        disp.drawLines(marker_lines2_world);

        std::vector<Vector2> cov_world = generateEllipse(x,y,sigma_xx,sigma_yy,sigma_xy);
        disp.setDrawColour(255,0,0);
        disp.drawLines(cov_world);

    }

    // Render GPS Measurements
    std::vector<std::vector<Vector2>> m_gps_marker = {{{0.5,0.5},{-0.5,-0.5}}, {{0.5,-0.5},{-0.5,0.5}}};
    disp.setDrawColour(255,255,255);
    for(const auto& meas : m_gps_measurement_history){disp.drawLines(offsetPoints(m_gps_marker, Vector2(meas.x,meas.y)));}

    // Render GPS Denied Zone
    if(m_sim_parameters.gps_denied_range > 0)
    {
        std::vector<Vector2> zone_lines = generateCircle(m_sim_parameters.gps_denied_x, m_sim_parameters.gps_denied_y, m_sim_parameters.gps_denied_range);
        disp.setDrawColour(255,150,0);
        disp.drawLines(zone_lines);
    }

    // Render Lidar Measurements
    for(const auto& meas : m_lidar_measurement_history)
    {
        double x0 = m_car.getVehicleState().x;
        double y0 = m_car.getVehicleState().y;
        double delta_x = meas.range * cos(meas.theta + m_car.getVehicleState().psi);
        double delta_y = meas.range * sin(meas.theta + m_car.getVehicleState().psi);
        disp.setDrawColour(201,201,0);
        disp.drawLine(Vector2(x0,y0), Vector2(x0 + delta_x,y0 + delta_y));
    }

    int x_offset, y_offset; 
    int stride = 20;
    // Simulation Status / Parameters
    x_offset = 10;
    y_offset = 30;
    std::string time_string = string_format("Time: %0.2f (x%d)",m_time,m_time_multiplier);
    std::string profile_string = string_format("Profile: %s", m_sim_parameters.profile_name.c_str());
    std::string gps_string = string_format("GPS: %s (%0.1f Hz)", (m_sim_parameters.gps_enabled ? "ON" : "OFF"), m_sim_parameters.gps_update_rate);
    std::string lidar_string = string_format("LIDAR: %s (%0.1f Hz)", (m_sim_parameters.lidar_enabled ? "ON" : "OFF"), m_sim_parameters.lidar_update_rate);
    std::string gyro_string = string_format("GYRO: %s (%0.1f Hz)", (m_sim_parameters.gyro_enabled ? "ON" : "OFF"), m_sim_parameters.gyro_update_rate);
    disp.drawText_MainFont(profile_string,Vector2(x_offset,y_offset+stride*-1),1.0,{255,255,255});
    disp.drawText_MainFont(time_string,Vector2(x_offset,y_offset+stride*0),1.0,{255,255,255});
    disp.drawText_MainFont(gps_string,Vector2(x_offset,y_offset+stride*1),1.0,{255,255,255});
    disp.drawText_MainFont(lidar_string,Vector2(x_offset,y_offset+stride*2),1.0,{255,255,255});
    disp.drawText_MainFont(gyro_string,Vector2(x_offset,y_offset+stride*3),1.0,{255,255,255});
    if (m_is_paused){disp.drawText_MainFont("PAUSED",Vector2(x_offset,y_offset+stride*4),1.0,{255,0,0});}
    if (!m_is_running){disp.drawText_MainFont("FINISHED",Vector2(x_offset,y_offset+stride*5),1.0,{255,0,0});}

    // Vehicle State
    x_offset = 800;
    y_offset = 10;
    std::string velocity_string = string_format("    Velocity: %0.2f m/s",m_car.getVehicleState().V);
    std::string yaw_string = string_format("   Heading: %0.2f deg",m_car.getVehicleState().psi * 180.0/M_PI);
    std::string xpos = string_format("X Position: %0.2f m",m_car.getVehicleState().x);
    std::string ypos = string_format("Y Position: %0.2f m",m_car.getVehicleState().y);
    disp.drawText_MainFont("Vehicle State",Vector2(x_offset-5,y_offset+stride*0),1.0,{255,255,255});
    disp.drawText_MainFont(velocity_string,Vector2(x_offset,y_offset+stride*1),1.0,{255,255,255});
    disp.drawText_MainFont(yaw_string,Vector2(x_offset,y_offset+stride*2),1.0,{255,255,255});
    disp.drawText_MainFont(xpos,Vector2(x_offset,y_offset+stride*3),1.0,{255,255,255});
    disp.drawText_MainFont(ypos,Vector2(x_offset,y_offset+stride*4),1.0,{255,255,255});

    std::string kf_velocity_string = string_format("    Velocity: %0.2f m/s",m_kalman_filter.getVehicleState().V);
    std::string kf_yaw_string = string_format("   Heading: %0.2f deg",m_kalman_filter.getVehicleState().psi * 180.0/M_PI);
    std::string kf_xpos = string_format("X Position: %0.2f m",m_kalman_filter.getVehicleState().x);
    std::string kf_ypos = string_format("Y Position: %0.2f m",m_kalman_filter.getVehicleState().y);
    disp.drawText_MainFont("Filter State",Vector2(x_offset,y_offset+stride*6),1.0,{255,255,255});
    disp.drawText_MainFont(kf_velocity_string,Vector2(x_offset,y_offset+stride*7),1.0,{255,255,255});
    disp.drawText_MainFont(kf_yaw_string,Vector2(x_offset,y_offset+stride*8),1.0,{255,255,255});
    disp.drawText_MainFont(kf_xpos,Vector2(x_offset,y_offset+stride*9),1.0,{255,255,255});
    disp.drawText_MainFont(kf_ypos,Vector2(x_offset,y_offset+stride*10),1.0,{255,255,255});

    // Keyboard Input
    x_offset = 10;
    y_offset = 650;
    disp.drawText_MainFont("Reset Key: r",Vector2(x_offset,y_offset+stride*0),1.0,{255,255,255});
    disp.drawText_MainFont("Pause Key: [space bar]",Vector2(x_offset,y_offset+stride*1),1.0,{255,255,255});
    disp.drawText_MainFont("Speed Multiplier (+/-) Key: [ / ] ",Vector2(x_offset,y_offset+stride*2),1.0,{255,255,255});
    disp.drawText_MainFont("Zoom (+/-) Key: + / - (keypad)",Vector2(x_offset,y_offset+stride*3),1.0,{255,255,255});
    disp.drawText_MainFont("Motion Profile Key: 1 - 9,0",Vector2(x_offset,y_offset+stride*4),1.0,{255,255,255});


    // Filter Error State
    x_offset = 750;
    y_offset = 650;
    std::string xpos_error_string = string_format("X Position RMSE: %0.2f m",calculateRMSE(m_filter_error_x_position_history));
    std::string ypos_error_string = string_format("Y Position RMSE: %0.2f m",calculateRMSE(m_filter_error_y_position_history));
    std::string heading_error_string = string_format("   Heading RMSE: %0.2f deg",180.0 / M_PI * calculateRMSE(m_filter_error_heading_history));
    std::string velocity_error_string = string_format("    Velocity RMSE: %0.2f m/s",calculateRMSE(m_filter_error_velocity_history));
    disp.drawText_MainFont(xpos_error_string,Vector2(x_offset,y_offset+stride*0),1.0,{255,255,255});
    disp.drawText_MainFont(ypos_error_string,Vector2(x_offset,y_offset+stride*1),1.0,{255,255,255});
    disp.drawText_MainFont(heading_error_string,Vector2(x_offset,y_offset+stride*2),1.0,{255,255,255});
    disp.drawText_MainFont(velocity_error_string,Vector2(x_offset,y_offset+stride*3),1.0,{255,255,255});
}
   
void Simulation::reset(SimulationParams sim_params){m_sim_parameters = sim_params; reset();}
void Simulation::increaseTimeMultiplier()
{
    m_time_multiplier++;
    std::cout << "Simulation: Time Multiplier Increased (x" << m_time_multiplier << ")" << std::endl;
}
void Simulation::decreaseTimeMultiplier()
{
    if (m_time_multiplier > 1)
    {
        m_time_multiplier--;
        std::cout << "Simulation: Time Multiplier Decreased (x" << m_time_multiplier << ")" << std::endl;
    }
}
void Simulation::setTimeMultiplier(unsigned int multiplier){m_time_multiplier = static_cast<int>(multiplier);}
void Simulation::increaseZoom()
{
    if (m_view_size > 25){m_view_size -= 25;}
    std::cout << "Simulation: Zoom Increased (" << m_view_size << "m)" << std::endl;
}
void Simulation::decreaseZoom()
{
    if (m_view_size < 400){m_view_size += 25;}
    std::cout << "Simulation: Zoom Decreased (" << m_view_size << "m)" << std::endl;
}
void Simulation::togglePauseSimulation()
{
    m_is_paused = !m_is_paused;
    std::cout << "Simulation: Paused (" << (m_is_paused?"True":"False") << ")" << std::endl;
}
bool Simulation::isPaused(){return m_is_paused;}
bool Simulation::isRunning(){return m_is_running;}
