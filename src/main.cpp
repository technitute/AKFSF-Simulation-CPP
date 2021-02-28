// System Includes
#include <string>
#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "simulation.h"
#include "car.h"
#include "display.h"

// Screen dimension constants
const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 768;
const double GRID_SIZE = 500;
const double GRID_SPACEING = 25;

// Function Prototypes
SimulationParams loadSimulation1Parameters();
SimulationParams loadSimulation2Parameters();
SimulationParams loadSimulation3Parameters();
SimulationParams loadSimulation4Parameters();
SimulationParams loadSimulation5Parameters();
SimulationParams loadSimulation6Parameters();
SimulationParams loadSimulation7Parameters();
SimulationParams loadSimulation8Parameters();
SimulationParams loadSimulation9Parameters();
SimulationParams loadSimulation0Parameters();

// Main Loop
int main( int argc, char* args[] )
{
    Display mDisplay;
    Simulation mSimulation;

    // Start Graphics
    if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
    {
        std::cout << "SDL could not initialize! SDL_Error: " <<  SDL_GetError() << std::endl;
        return -1;
    }
    if( TTF_Init() == -1 )
    {
        std::cout << "SDL_ttf could not initialize! SDL_ttf Error: " << TTF_GetError() << std::endl;
        return -1;
    }

    // Create Display
    if (!mDisplay.createRenderer("AKFSF Simulations", SCREEN_WIDTH, SCREEN_HEIGHT)){return false;}

    // Main Simulation Loop
    mSimulation.reset(loadSimulation1Parameters());
    //mSimulation.setTimeMultiplier(10);
    bool mRunning = true;
    while(mRunning)
    {
        // Update Simulation
        mSimulation.update();

        // Update Display
        mDisplay.clearScreen();

            // Draw Background Grid
            mDisplay.setDrawColour(101,101,101);
            for (int x = -GRID_SIZE; x <= GRID_SIZE; x+=GRID_SPACEING){mDisplay.drawLine(Vector2(x,-GRID_SIZE),Vector2(x,GRID_SIZE));}
            for (int y = -GRID_SIZE; y <= GRID_SIZE; y+=GRID_SPACEING){mDisplay.drawLine(Vector2(-GRID_SIZE,y),Vector2(GRID_SIZE,y));}

            // Draw Simulation
            mSimulation.render(mDisplay);

        mDisplay.showScreen();

        // Handle Events
        SDL_Event event;
        while( SDL_PollEvent( &event ) != 0 )
        {
            if( event.type == SDL_QUIT ){mRunning = false;}
            else if (event.type == SDL_KEYDOWN)
            {
                switch( event.key.keysym.sym )
                {               
                    case SDLK_SPACE: mSimulation.togglePauseSimulation(); break;
                    case SDLK_ESCAPE:mRunning = false; break;
                    case SDLK_KP_PLUS: mSimulation.increaseZoom(); break;
                    case SDLK_KP_MINUS: mSimulation.decreaseZoom(); break;
                    case SDLK_RIGHTBRACKET: mSimulation.increaseTimeMultiplier(); break;
                    case SDLK_LEFTBRACKET: mSimulation.decreaseTimeMultiplier(); break;
                    case SDLK_r: mSimulation.reset(); break;
                    case SDLK_1: mSimulation.reset(loadSimulation1Parameters()); break;
                    case SDLK_2: mSimulation.reset(loadSimulation2Parameters()); break;
                    case SDLK_3: mSimulation.reset(loadSimulation3Parameters()); break;
                    case SDLK_4: mSimulation.reset(loadSimulation4Parameters()); break;
                    case SDLK_5: mSimulation.reset(loadSimulation5Parameters()); break;
                    case SDLK_6: mSimulation.reset(loadSimulation6Parameters()); break;
                    case SDLK_7: mSimulation.reset(loadSimulation7Parameters()); break;
                    case SDLK_8: mSimulation.reset(loadSimulation8Parameters()); break;
                    case SDLK_9: mSimulation.reset(loadSimulation9Parameters()); break;
                    case SDLK_0: mSimulation.reset(loadSimulation0Parameters()); break;
                }
            }
        }
    }

    // Destroy Renderer
    mDisplay.destroyRenderer();

    // Unload SDL
    TTF_Quit();
    SDL_Quit();

    return 0;
}

SimulationParams loadSimulation1Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "1 - Constant Velocity + GPS + GYRO + Zero Initial Conditions";
    sim_params.car_initial_velocity = 5;
    sim_params.car_initial_psi = M_PI/180.0 * 45.0;
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(500,500,5));
    return sim_params;
}

SimulationParams loadSimulation2Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "2 - Constant Velocity + GPS + GYRO + Non-zero Initial Conditions";
    sim_params.car_initial_x = 500;
    sim_params.car_initial_y = 500;
    sim_params.car_initial_velocity = 5;
    sim_params.car_initial_psi = M_PI/180.0 * -135.0;
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(0,0,5));
    return sim_params;
}

SimulationParams loadSimulation3Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "3 - Constant Speed Profile + GPS + GYRO";
    sim_params.car_initial_velocity = 5;
    sim_params.car_initial_psi = M_PI/180.0 * 45.0;
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(100,100,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(100,-100,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(0,100,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(0,0,5));
    return sim_params;
}

SimulationParams loadSimulation4Parameters()
{    
    SimulationParams sim_params;
    sim_params.profile_name = "4 - Variable Speed Profile + GPS + GYRO";
    sim_params.end_time = 200;
    sim_params.car_initial_velocity = 0;
    sim_params.car_initial_psi = M_PI/180.0 * 45.0;
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(100,100,2));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(100,-100,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(0,100,7));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(0,0,2));
    return sim_params;
}

SimulationParams loadSimulation5Parameters()
{    
    SimulationParams sim_params = loadSimulation1Parameters();
    sim_params.profile_name = "5 - Constant Velocity + GPS + GYRO + LIDAR+ Zero Initial Conditions";
    sim_params.lidar_enabled = true;
    return sim_params;
}

SimulationParams loadSimulation6Parameters()
{    
    SimulationParams sim_params = loadSimulation2Parameters();
    sim_params.profile_name = "6 - Constant Velocity + GPS + GYRO + LIDAR + Non-zero Initial Conditions";
    sim_params.lidar_enabled = true;
    return sim_params;
}

SimulationParams loadSimulation7Parameters()
{    
    SimulationParams sim_params = loadSimulation3Parameters();
    sim_params.profile_name = "7 - Constant Speed Profile + GPS + GYRO + LIDAR";
    sim_params.lidar_enabled = true;
    return sim_params;
}


SimulationParams loadSimulation8Parameters()
{    
    SimulationParams sim_params = loadSimulation4Parameters();
    sim_params.profile_name = "8 - Variable Speed Profile + GPS + GYRO + LIDAR";
    sim_params.lidar_enabled = true;
    return sim_params;
}

SimulationParams loadSimulation9Parameters()
{    
    SimulationParams sim_params;
    sim_params.profile_name = "9 - CAPSTONE";
    sim_params.gyro_enabled = true;
    sim_params.lidar_enabled = true;
    sim_params.end_time = 500;
    sim_params.car_initial_x = 400;
    sim_params.car_initial_y = -400;
    sim_params.car_initial_velocity = 0;
    sim_params.car_initial_psi = M_PI/180.0 * -90.0;
    sim_params.gps_error_probability = 0.05;
    sim_params.gps_denied_x = 250.0;
    sim_params.gps_denied_y = -250.0;
    sim_params.gps_denied_range = 100.0;
    sim_params.gyro_bias = -3.1/180.0*M_PI;
    sim_params.car_commands.emplace_back(new MotionCommandStraight(3,-2));
    sim_params.car_commands.emplace_back(new MotionCommandTurnTo(M_PI/180.0 * 90.0,-2));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(400,-300,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(350,-300,2));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(300,-250,7));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(300,-300,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(250,-250,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(250,-300,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(200,-250,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(200,-300,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(200,-150,2));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(100,-100,-2));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(200,0,7));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(300,-100,5));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(300,-300,7));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(400,-300,3));
    sim_params.car_commands.emplace_back(new MotionCommandMoveTo(400,-400,1));
    return sim_params;
}

SimulationParams loadSimulation0Parameters()
{    
    SimulationParams sim_params = loadSimulation9Parameters();
    sim_params.profile_name = "0 - CAPSTONE BONUS (with No Lidar Data Association)";
    sim_params.lidar_id_enabled = false;
    return sim_params;
}