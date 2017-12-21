# Self-Driving Car Engineer Nanodegree

## Term 2 Project 5: Model Predictive Control (MPC)

The goal for this project was to implement a MPC to drive a car in the SDC simulator. The project specification was given [here](https://review.udacity.com/#!/rubrics/896/view).

### Approach

The concept of a Model Predictive Controller is to use the odometry of the car and a set of waypoints along a track to be driven. This data is provided by the SDC simulator.




The waypoints are covverted into the vecicle coordinates
```
double dx = ptsx[i] - px;
double dy = ptsy[i] - py;
ptsx[i] = dx * cos(-psi) - dy * sin(-psi);
ptsy[i] = dx * sin(-psi) + dy * cos(-psi);
```




[SDC simulator] --> providing (A) odometry and (B) waypoints --> [MPC] --> converting (B) waypoints to vehicle coordinates -->



```
adgadg
```


### Challenges



### Conclusion

This project is mostly about tuning the parameters of the system. The current setting does drive the course, but is pretty far from a smooth ride. It needs a lot more time tuning and not just by trial and error. If the PID controller should be used a tuning system like twiddle would be helpful.

