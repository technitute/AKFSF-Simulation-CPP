#ifndef INCLUDE_AKFSFSIM_CAR_H
#define INCLUDE_AKFSFSIM_CAR_H

#include <queue>
#include <cmath>
#include "display.h"
#include "utils.h"

struct VehicleState
{
    double x,y,psi,V;
    double yaw_rate,steering;
    VehicleState():x(0.0), y(0.0), psi(0.0), V(0.0), yaw_rate(0.0), steering(0.0) {}
    VehicleState(double setX, double setY, double setPsi, double setV):x(setX), y(setY), psi(setPsi), V(setV), yaw_rate(0.0),steering(0.0) {}
    VehicleState(double setX, double setY, double setPsi, double setV, double setPsiDot, double setSteering):x(setX), y(setY), psi(setPsi), V(setV), yaw_rate(setPsiDot),steering(setSteering) {}
};

class MotionCommandBase
{
    public:
        MotionCommandBase():m_velocity_command(0.0),m_steering_command(0.0){}
        virtual void startCommand(double time, VehicleState state){m_start_time = time;m_start_state = state;}
        virtual void endCommand(double time, double dt, VehicleState state){}
        virtual bool update(double time, double dt, VehicleState state){return false;}
        virtual double getVelocityCommand(){return m_velocity_command;}
        virtual double getSteeringCommand(){return m_steering_command;}
    protected:
        double m_velocity_command, m_steering_command, m_start_time;
        VehicleState m_start_state;
};

class MotionCommandStraight : public MotionCommandBase
{
    public:
    MotionCommandStraight(double command_time, double command_velocity):m_command_time(command_time),m_command_velocity(command_velocity){}
    bool update(double time, double dt, VehicleState state)
    {
        m_velocity_command = m_command_velocity;
        m_steering_command = 0.0;
        return time > (m_start_time + m_command_time);
    }
    private:
        double m_command_time, m_command_velocity;
};

class MotionCommandTurnTo : public MotionCommandBase
{
    public:
    MotionCommandTurnTo(double command_heading, double command_velocity):m_command_heading(command_heading),m_command_velocity(command_velocity){}
    bool update(double time, double dt, VehicleState state)
    {
        m_velocity_command = m_command_velocity;
        double angle_error = wrapAngle(m_command_heading - state.psi);
        m_steering_command = angle_error * (std::signbit(state.V)?-1.0:1.0);
        return std::fabs(angle_error) < 0.001;
    }
    private:
        double m_command_heading, m_command_velocity;
};

class MotionCommandMoveTo : public MotionCommandBase
{
    public:
    MotionCommandMoveTo(double command_x, double command_y, double command_velocity):m_command_x(command_x),m_command_y(command_y),m_command_velocity(command_velocity){}
    bool update(double time, double dt, VehicleState state)
    {
        m_velocity_command = m_command_velocity;
        double delta_x = m_command_x - state.x;
        double delta_y = m_command_y - state.y;
        double range = sqrt(delta_x*delta_x + delta_y*delta_y);
        double angle_command = atan2(delta_y,delta_x);
        double psi = wrapAngle(state.psi - (std::signbit(state.V) ?M_PI:0.0));
        double angle_error = wrapAngle(angle_command - psi);
        m_steering_command = angle_error * (std::signbit(state.V)?-1.0:1.0);
        return (range < 5.0);
    }
    private:
        double m_command_x, m_command_y, m_command_velocity;
};

class BicycleMotion
{
    public:

        BicycleMotion():m_initial_state(VehicleState(0,0,0,0)),m_wheel_base(4.0),m_max_velocity(28.0),m_max_acceleration(2.0),m_max_steering(0.8) {reset();}
        BicycleMotion(double x0, double y0, double psi0, double V0):m_initial_state(VehicleState(x0,y0,psi0,V0)),m_wheel_base(4.0),m_max_velocity(28.0),m_max_acceleration(2.0),m_max_steering(0.8){reset();}

        void reset()
        {
            m_current_state = m_initial_state;
            m_steering_command = m_initial_state.steering;
            m_velocity_command = m_initial_state.V;
        }
        void reset(VehicleState state)
        {
            m_initial_state = state;
            reset();
        }

        void update(double dt)
        {
            double cosPsi = cos(m_current_state.psi);
            double sinPsi = sin(m_current_state.psi);
            double x = m_current_state.x + m_current_state.V * cosPsi * dt;
            double y = m_current_state.y + m_current_state.V * sinPsi * dt;

            double accel = m_velocity_command - m_current_state.V;
            if (accel > m_max_acceleration) {accel = m_max_acceleration;}
            if (accel < -m_max_acceleration) {accel = -m_max_acceleration;}

            double steer = m_steering_command;
            if (steer > m_max_steering) {steer = m_max_steering;}
            if (steer < -m_max_steering) {steer = -m_max_steering;}

            double vel = m_current_state.V + accel * dt;
            if (vel > m_max_velocity) {vel = m_max_velocity;}
            if (vel < -m_max_velocity) {vel = -m_max_velocity;}

            double psi_dot = m_current_state.V*steer/m_wheel_base;
            double psi = wrapAngle(m_current_state.psi + psi_dot* dt);
            m_current_state = VehicleState(x,y,psi,vel,psi_dot,steer);
        }

        void setSteeringCmd(double steer){m_steering_command = steer;}
        void setVelocityCmd(double accel){m_velocity_command = accel;}
        VehicleState getVehicleState() const {return m_current_state;}

    private:

        VehicleState m_current_state;
        VehicleState m_initial_state;

        double m_steering_command;
        double m_velocity_command;

        double m_wheel_base;

        double m_max_velocity;
        double m_max_acceleration;
        double m_max_steering;

};

class Car
{
    public:

        Car():m_vehicle_model(),m_current_command(nullptr)
        {
            // Create Display Geometry
            m_car_lines_body = {{2,-1},{2,1},{-2,1},{-2,-1},{2,-1}};
            m_marker_lines = {{{0.5,0.5},{-0.5,-0.5}}, {{0.5,-0.5},{-0.5,0.5}}, {{0,0},{3.5,0}}};
            m_wheel_lines = {{-0.6,0.3},{0.6, 0.3},{0.6, -0.3},{-0.6, -0.3},{-0.6, 0.3}};
            m_wheel_fl_offset = Vector2(2, -1.6);
            m_wheel_fr_offset = Vector2(2, 1.6);
            m_wheel_rl_offset = Vector2(-2, -1.6);
            m_wheel_rr_offset = Vector2(-2, 1.6);
        }

        void reset(double x0, double y0, double psi0, double V0)
        {
            m_vehicle_model.reset(VehicleState(x0,y0,psi0,V0));
            while (!m_vehicle_commands.empty()){m_vehicle_commands.pop();}
            m_current_command = nullptr;
        }

        void addVehicleCommand(MotionCommandBase* cmd)
        {
            if (cmd != nullptr){m_vehicle_commands.push(cmd);}
        }

        VehicleState getVehicleState() const {return m_vehicle_model.getVehicleState();}

        bool update(double time, double dt)
        {
            // Update Command
            if(m_current_command == nullptr && !m_vehicle_commands.empty())
            {
                m_current_command = m_vehicle_commands.front();
                m_vehicle_commands.pop();
                m_current_command->startCommand(time, m_vehicle_model.getVehicleState());
            }

            // Run Command
            if (m_current_command != nullptr)
            {
                bool cmd_complete = m_current_command->update(time, dt, m_vehicle_model.getVehicleState());
                m_vehicle_model.setSteeringCmd(m_current_command->getSteeringCommand());
                m_vehicle_model.setVelocityCmd(m_current_command->getVelocityCommand());
                if(cmd_complete){m_current_command = nullptr;}
            }
            else
            {
                m_vehicle_model.setSteeringCmd(0.0);
                m_vehicle_model.setVelocityCmd(0.0);
            }

            // Update Vehicle
            m_vehicle_model.update(dt);

            return true;
        }

        void render(Display& disp)
        {
            double steeringPsi = m_vehicle_model.getVehicleState().steering;
            double carPsiOffset = m_vehicle_model.getVehicleState().psi;
            Vector2 carPosOffset = Vector2(m_vehicle_model.getVehicleState().x, m_vehicle_model.getVehicleState().y);
            
            disp.setDrawColour(0,255,0);
            disp.drawLines(transformPoints(m_car_lines_body, carPosOffset, carPsiOffset));
            disp.drawLines(transformPoints(m_marker_lines, carPosOffset, carPsiOffset));

            disp.setDrawColour(0,201,0);
            disp.drawLines(transformPoints(transformPoints(m_wheel_lines, m_wheel_fl_offset, steeringPsi), carPosOffset, carPsiOffset));
            disp.drawLines(transformPoints(transformPoints(m_wheel_lines, m_wheel_fr_offset, steeringPsi), carPosOffset, carPsiOffset));
            disp.drawLines(transformPoints(offsetPoints(m_wheel_lines, m_wheel_rl_offset), carPosOffset, carPsiOffset));
            disp.drawLines(transformPoints(offsetPoints(m_wheel_lines, m_wheel_rr_offset), carPosOffset, carPsiOffset));
        }

    private:

        BicycleMotion m_vehicle_model;
        MotionCommandBase* m_current_command;
        std::queue<MotionCommandBase*> m_vehicle_commands;

        std::vector<Vector2> m_car_lines_body;
        std::vector<Vector2> m_wheel_lines;
        std::vector<std::vector<Vector2>> m_marker_lines;
        Vector2 m_wheel_fl_offset, m_wheel_fr_offset, m_wheel_rl_offset, m_wheel_rr_offset;
};


#endif  // INCLUDE_AKFSFSIM_CAR_H