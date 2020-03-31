# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

## Important Dependencies
1. The Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).
2. uWebSocketIO: The repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. 
3. cmake >= 3.5
4. make >= 4.1 (Linux, Mac), 3.81 (Windows)
5. gcc/g++ >= 5.4

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF`

## Project Overview

### Input to the Filter
The input data consisted of the following columns: Raw Measurements, Timestamp and Ground Truth values of the positions and velocities of the object to be tracked. The raw measurements are the noisy measurement values taken from Lidar and Radar. They are positions in X and Y axis (px and py) for the lidar measurements and the radius (rho) , azimuth angle (psi) and the velocity (rho_dot) for the radar measurements. The timestamp is in microseconds and denotes the timestamp the data was captured. The Ground Truth values are the actual values of the positions and velocities of the object in the X and Y axes (px, py, vx, vy) and is to be used for comparison with the estimated values of the same obtained from the Kalman filter.

### Output from the Filter
The Output of the Unscented Kalman Filter are the estimated values of the state matrix consisting of the positions in x and y direction, velocity, yaw and yaw rate of the object. The position estimates are fed into the simulator for marking and are also compared with the ground truth values to evaluate the performance of the filter.

### The Unscented Kalman Filter
The Unscented Kalman Filter is similar to the Extended Kalman Filter in that it has the same inputs and outputs and consists of prediction and update steps. However, instead of linearizing around a fixed point to take care of the non-linearity, the Unscented Kalman Filter uses Sigma Points. The Sigma-Points are generated based on the mean and co-variance. These Sigma points are then passed on for Prediction and Update steps and the consequent means and co-variances are then back calculated from the "Predicted" or "Updated" sigma points. 

#### Prediction step: 
The prediction step predicts the values of the states (px, py, v, yaw and yawd) based on the state transition equations. The state co-variance matrix is updated based on the process noise of the prediction step. Once it receives the mean and co-variance of the state matrix, the sigma points for the states are generated using the equations below. 
  
  X_{k|k} = [ x_{k|k} x_{k|k}+sqrt{(lambda+n_x)*P_{k|k}} x_{k|k}-sqrt{(lambda+n_x)P_{k|k}} ]
                
where lambda is (3- n_x), n_x is the number of states and P is the state co-variance matrix. The number of sigma points are 2*n_x + 1. 

Next, each of these sigma points are passed through the state transition equations to generate the new mean and the co-variance.
                        x' = x +F + v
                        P' = [P 0
                              0 Q]
                          
Where F is the state transition matrix, v is the process noise matrix, Q is the process co-variance matrix which in this case is the variance of longitudnal acceleration and yaw acceleration.

Once these new sigma points are obtained,the new mean and co-variance is obtained from these sigma points by summing them according to their weights.
                
                x_{k+1|k} = sum(w_i * X_{k+1|k,i})

the w_i is the weight matrix which is calculated as:

                    w_0 = lambda/(lambda+n_x)
                    w_1...w_n = 1/(2*lambda)

####Update Step:
The measurements obtained from the Lidar or the Radar are then used to update the predicted states. The lidar update follows the same Kalman Filter equations as before since the measurement matrix of Lidar is linear. Since the Radar measurements are in polar co-ordinates and the states are in cartesian co-ordinates, the measurement matrix for the Radar is non-linear in nature. Hence, an unscented Kalman filter is again used for the radar measurements which involves generating the sigma points and passing them individually through the update steps. The following are the equations for the update step for Lidar measurement update:

				        y = z - H * x
                S = H * P * H' + R
                K = P * H' * S.inverse
                x = x + K * y
                P = (I - K * H) * P
                
For the Radar measurements, the sigma points from the prediction step can be carried forward so we do not have to generate the sigma points again. A measurement prediction is made based on the sigma points and the difference of the actual measurement from the predicted measurement is used to calculate the Kalman and update the co-variance matrix.
                        S = sum(w_i * x_diff * x_diff') + R
                        T = sum(w_i * x_diff * z_diff')   
                        K = T * (S ^ -1)
                        x = x + K * z_diff
                        P = P - K * S * K'
where x_diff is the difference between the sigma point and the states, z_diff is the difference between the actual measurements and the measurement sigma point, R is the noise matrix for the sensor and K is the Kalman gain.  

The UKF.cpp file has the  has the generic flow of the algorithm and the functions for the above mentioned Prediction and Update steps. The Main.cpp file is used to interact with the simulator.

![](./Images/Simulator_1.png)

![](./Images/Simulator_2.png)

## Evaluation

The RMSE values were found to be less than [.1, .1, 0.52, 0.52] as dictated by the project rubric to pass the project. 

