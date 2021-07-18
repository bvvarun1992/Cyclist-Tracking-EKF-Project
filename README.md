# Tracking using Extended Kalman Filter
---

## Overview

This project involves utilizing a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. The RMSE values are computed against the ground truth to judge the accuracy of the Kalman filter estimation.

---
## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
---
## Instructions

This project uses the Simulator developed by Udacity which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). **uWebSocketIO** is used to communicate with the simulator.

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems.
For Linux installation, run the script `./install-ubuntu.sh` and for mac installation, run thje script `install-mac.sh`

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

A successful build must return `Listening to port 4567` in the terminal which means that the communication with the simulator is established. The results of the state estimated by EKF can now be visualized by running the simulator. 

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]
