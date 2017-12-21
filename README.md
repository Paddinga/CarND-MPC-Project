# Self-Driving Car Engineer Nanodegree

## Term 2 Project 5: Model Predictive Control (MPC)

The goal for this project was to implement a MPC to drive a car in the SDC simulator. The project specification was given [here](https://review.udacity.com/#!/rubrics/896/view).

### Approach

The concept of a Model Predictive Controller is to use the odometry of the car and a set of waypoints along a track to be driven. This data is provided by the SDC simulator and handed to the MPC running through the following model to predict the trajectory the car needs to drive to drive along the waypoints. The calculated values for steering and throttle are handed back to the simulator. This process is repeated with the next set of data provided by the simulator. In the code the delay of 100ms is also taken acoount of.

1. Predict odometry for a delay (dt) of 100ms

The vehicle state is forecasted calculating the future position px and py and the direction of the car considering the steering angle (delta). Also the future speed is calculated considering the current throttle (a) with a empirical correction factor of 10 due to missing information about the actual acceleration. The current values for the steering angle and throttle are also handed by the SDC simulator alongside the odometry data.
```
px = px + v * cos(psi) * dt;
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

A 3rd order polynomial is generated along the waypoints. Additionally the cross track error (CTE) and the orientation (EPSI) are calculated. The vehicle speed from 1., the cte and the epsi are combined to the vehicle state and handed to the MPC.

4. Initializing and running the MPC

The MPC is initialized with a horizon of 10 steps and a interim time of 100ms. So the prediction horizon of the MPC T is set so 1 second.

To take account of several influences to the overall cost function the parts of the cost function are defined. The main directive for the MPC is to follow the handed trajectory. Therefore the cost for the CTE and the EPSI are considered with the power of 6 and a factor of 100. The target speed ref_v is set to 40 mph, but since I actually don't care much the vehicle will accerelate and drive as fast as possible (max speed around 85 mph) and the cost for the speed is not a big part of the overall cost function with a factor of 1. To get the car driving quite smoothly I decided to give the use a factor of 1000 for usage of steering and the gap between to sequential steering angles.
```
/ Cost funtion
// The part of the cost based on the reference state.
for (int i = 0; i < N; i++) {
fg[0] += 100 * CppAD::pow(vars[cte_start + i] - ref_cte, 6);
fg[0] += 100 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 6);
fg[0] += 1 * CppAD::pow(vars[v_start + i] - ref_v, 2);
}

// Minimize the use of actuators.
for (int i = 0; i < N - 1; i++) {
fg[0] += 1000 * CppAD::pow(vars[delta_start + i], 2);
fg[0] += 1 * CppAD::pow(vars[a_start + i], 2);
}

// Minimize the value gap between sequential actuations.
for (int i = 0; i < N - 2; i++) {
fg[0] += 1000 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
fg[0] += 10 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}
```

The state between two timesteps is described by the following equations based on the course.
```
AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```

5. Calculate steering angle and throttle and hand to SDC simulator

The MPC calculated based on the shown model the best steering angle and value for throttle to keep the car along the trajectory. These values are handed to the SDC simulator.


### Challenges

There were two major challenges during this project.

1. Implementing the delay

First I tried to implement the delay after calculating the trajectory and just hand this values to the MPC::Solve, but the car did not drive well. Instead of taking the curves with acceptable speed it was very slow and still driving over the edges of the road. Consulting a friend he gave me the hint implementing the delay right before creating the waypoint vector. This worked really well.

2. Tuning the cost function

Tuning the cost funtion took quite some time and it is still not perfect. But the vehicle speed is fine, all curves are driven without cutting the edges and even the s-curve is handled. I am not happy how it drives straight or curves with big radiuses because it is still oscilliating. The biggest step achieving the current driving behaviour was to add the power of 6 to the cost of CTE and EPSI.


### Conclusion

This project was quite complex implementing the model and the MPC, but I learned a lot about the behaviour of model prediction for vehicles.‚

