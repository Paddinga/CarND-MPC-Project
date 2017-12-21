# Self-Driving Car Engineer Nanodegree

## Term 2 Project 5: Model Predictive Control (MPC)

The goal for this project was to implement a MPC to drive a car in the SDC simulator. The project specification was given [here](https://review.udacity.com/#!/rubrics/896/view).

### Approach

The concept of a Model Predictive Controller is to use the odometry of the car and a set of waypoints along a track to be driven. This data is provided by the SDC simulator and handed to the MPC running through the following model to predict the trajectory the car needs to drive to drive along the waypoints. The calculated values for steering and throttle are handed back to the simulator. This process is repeated with the next set of data provided by the simulator. In the code the delay of 100ms is also taken acoount of.

1. Predict odometry for a delay (dt) of 100ms
The vehicle state is forecasted calculating the future position px and py and the direction of the car considering the steering angle (delta). Also the future speed is calculated considering the current throttle (a) with a empirical correction factor of 10 due to missing information about the actual acceleration. The current values for the steering angle and throttle are also handed by the SDC simulator alongside the odometry data.
```px = px + v * cos(psi) * dt;
py = py + v * sin(psi) * dt;
psi = psi + v / Lf * -delta * dt;
v = v + a * 10 * dt;
```

2. The waypoints are converted into the vecicle coordinates
```
double dx = ptsx[i] - px;
double dy = ptsy[i] - py;
ptsx[i] = dx * cos(-psi) - dy * sin(-psi);
ptsy[i] = dx * sin(-psi) + dy * cos(-psi);
```

3. Fitting polynomial and calculation state
A 3rd order polynomial is generated along the waypoints. Additionally the cross track error (CTE) and the orientation (epsi) are calculated. The vehicle speed from 1., the cte and the epsi are combined to the vehicle speed.

4.



### Challenges



### Conclusion

This project is mostly about tuning the parameters of the system. The current setting does drive the course, but is pretty far from a smooth ride. It needs a lot more time tuning and not just by trial and error. If the PID controller should be used a tuning system like twiddle would be helpful.

