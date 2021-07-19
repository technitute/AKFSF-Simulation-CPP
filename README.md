# Advanced Kalman Filtering and Sensor Fusion Simulation #

Welcome to the Advanced Kalman Filtering and Sensor Fusion Simulation exercise project. In this project, you will be developing the source code for a number of different types of Kalman Filters which are used to estimate the navigation state of a 2D vehicle problem; such as that would be found on a self-driving car!


![AKFSF-Simulation](/AKFSF-Simulation.gif)


This README is broken down into the following sections:

- [Setup](#setup) - the environment and code setup required to get started and a brief overview of the project structure.
 - [The Tasks](#the-tasks) - the tasks you will need to complete for the project

 ## Setup ##

This project will use the Ubuntu 64 20.04.2.0 LTS VM C++ development environment that is setup for this course. (Refer to the Setting up the Development Environment Lesson) Please follow the steps below to compile the simulation.

 0. Install the dependencies
 ```
 sudo apt install libeigen3-dev libsdl2-dev libsdl2-ttf-dev
 ```
 
 1. Clone the repository
 ```
 git clone https://github.com/technitute/AKFSF-Simulation-CPP.git
 ```
 2. Setup the cmake build
 ```
 cd AKFSF-Simulation-CPP
 mkdir build
 cd build
 cmake ../
 ```

 3. Compile the code
 ```
 make
 ```
 
 3. You should now be able to and run the estimation simulator
 ```
 ./AKFSF-Simulation
 ```
### Project Structure ###
There are 6 main files of interest:
* kalmanfilter_lkf_student.cpp
* kalmanfilter_ekf_student.cpp
* kalmanfilter_ukf_student.cpp
* kalmanfilter_lkf_answer.cpp
* kalmanfilter_ekf_answer.cpp
* kalmanfilter_ukf_answer.cpp

To use one of the filter files, replace the ```kalmanfilter.cpp``` file with the one of interest.

When carrying out the Linear Kalman Filter task, use the ```kalmanfilter_lkf_student.cpp``` as a base and develop all the exercise on top of each other. The file ```kalmanfilter_lkf_answer.cpp``` is a working version that you can use to compare your solution too. The same thing applies to the Extended Kalman Filter and Unscented Kalman Filter exercises with the corresponding files. You can use any of the files as a base for your capstone project.

## Simulation Operation ##
The simulation can be run with different motion and sensor profiles to test the different scenarios and evaluate how the filter performs with different conditions. The different profiles can be activated pressing the number keys 1-9,0, to active the corresponding profile.

### Motion Profiles ###
* 1 - Constant Velocity + GPS + GYRO + Zero Initial Conditions
* 2 - Constant Velocity + GPS + GYRO + Non-zero Initial Conditions
* 3 - Constant Speed Profile + GPS + GYRO
* 4 - Variable Speed Profile + GPS + GYRO
* 5 - Constant Velocity + GPS + GYRO + LIDAR+ Zero Initial Conditions
* 6 - Constant Velocity + GPS + GYRO + LIDAR + Non-zero Initial Conditions
* 7 - Constant Speed Profile + GPS + GYRO + LIDAR
* 8 - Variable Speed Profile + GPS + GYRO + LIDAR
* 9 - CAPSTONE
* 0 - CAPSTONE BONUS (with No Lidar Data Association)



## The Tasks ##
The tasks for this simulator can be broken down into 4 different areas:
1. Linear Kalman Filter
2. Extended Kalman Filter
3. Unscented Kalman Filter
4. Capstone Project

### Linear Kalman Filter (LKF Exercise 1)
Starting with the ```kalmanfilter_lkf_student.cpp``` and replace the ```kalmanfilter.cpp``` file. Initialize the filter on first prediction step and then implement the 2D Vehicle process model and Linear Kalman Filter Prediction steps. Test and verify good performance with profile 1.

### Linear Kalman Filter (LKF Exercise 2)
Continuing on from Exercise 2, Implement the GPS update step using the Linear Kalman Filter Update equations. Test and verify good performance with profile 1. Play around with tunings and see results for profiles 2,3 and 4.

### Linear Kalman Filter (LKF Exercise 3)
Continuing on from Exercise 3, Modify the filter to initialize the filter on first GPS measurement rather than prediction. Use the flag ```INIT_ON_FIRST_PREDICTION = false``` to enable this functionality. Test and verify good performance with profile 1 and 2.

### Extended Kalman Filter (EKF Exercise 1)
Starting with the ```kalmanfilter_ekf_student.cpp``` and replace the ```kalmanfilter.cpp``` file. 
Implement the 2D Vehicle process model and Extended Kalman Filter Prediction steps. Test and verify good performance with profiles 1-4.

### Extended Kalman Filter (EKF Exercise 2)
Continuing on from EKF Exercise 2, Implement the LIDAR update step using the Extended Kalman Filter Update equations. Test and verify good performance with profiles 1-8.

### Unscented Kalman Filter (UKF Exercise 1)
Starting with the ```kalmanfilter_ukf_student.cpp``` and replace the ```kalmanfilter.cpp``` file. 
Implement the 2D Vehicle Process model and Unscented Kalman Filter Prediction steps. Test and verify good performance with profiles 1-4.

### Unscented Kalman Filter (UKF Exercise 2)
Continuing on from UKF Exercise 2, Implement the LIDAR update step using the Unscented Kalman Filter Update equations. Test and verify good performance with profiles 1-8.

### Capstone Project
Starting from any filter base replace the ```kalmanfilter.cpp``` file. Program a filter to provide the best estimation performance for profiles 9,0 (It should also work on any other profiles aswell).

## Authors ##

This project was developed for the Technitute Course - Advanced Kalman Filtering and Sensor Fusion. Developed and produced by Dr. Steven Dumble.