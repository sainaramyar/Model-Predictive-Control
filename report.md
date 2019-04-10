
# Project Overview

The objective of this project is to plan the optimal trajectory of a vehicle on a track using model predictive control.

## Vehicle Model

The vehicle model used in this project is a kinematic model. This model has four states which are longitudinal and lateral position (`x` and `y`), velocity `v` and orientation angle `psi`. The model also has two control inputs acceleration `a` and steering angle `delta`.

The system states are updated as follows:

`x1 = (x0 + v0 * CppAD::cos(psi0) * dt)
y1 = (y0 + v0 * CppAD::sin(psi0) * dt)
psi1 = (psi0 + v0 * delta0 / Lf * dt)
v1 = (v0 + a0 * dt)`

Where `Lf` is the length of the vehicle.
  


## Control Horizon and Sampling Time

The MPC determines an optimal control signal over a finite horizon. The control horizon (`N`) and time steps (`dt`) within the horizon are chosen by the designer. Choosing values for both these parameters is a trade off between accuracy and computational cost.

A larger value for `N` results in prediction of a longer time in the future, but it also increases the computational load which may decrease the real-time performance of the controller. On the other hand, an unusally large horizon may also result in predictions far into the future which are unrealistic for the controller to process. 

A similar challenge exists for choosing `dt`, a smaller value for sampling time increases the accuracy of prediction, but limits the horizon for a fixed (`N`) value.

Overall, a combination of `N` and `dt` that results in a predicion horizon of around 2 seconds is ideal in my opinion, because it is long enough for the controller to plan the optimal trajectory but not too long that it misses something unexpectant along the way. As a result, `N = 10` and `dt = 0.2` were used for the project.
I experimented with larger `N` values  such as 15 and 20, and turned out 20 is too large fot the horzion and 15 did not showed any change in performance so they were not chosen. In addition, I chose the value of `dt` larger than the latency in order to reduce its effect.

## Coordinate Transforms

Since the controller works in the vehicle coordinate system, all the values in the map coordinate should be transformed. In the car coordinate, the x axis is in the vehicle's direction and y axis is orthogonal to the x, with positive values to the left. The transformation equations are as follows:

`for (int i = 0; i < ptsx.size(); i++) {
			  double x = ptsx[i] - px;
			  double y = ptsy[i] - py;
			  ptsx1[i] = (x * cos(psi) + y * sin(psi));
			  ptsy1[i] = (-x * sin(psi) + y * cos(psi));
		  }`
          
Where `ptsx` and `ptsy` are the reference trajectory, `px` and `py` are the measures positions from the vehicle and `ptsx1` and `ptsy1` are the transformed coordinates.
          

## Latency

In this project there is a 100 millisecond latency between actuations commands on top of the connection latency.
In order to compensat efor this latency, the behavior of the vehicle is estimated over that time and added to the measeured vale. The following equations are used for compensation:

`v = v + a*latency;
px = px + v*cos(psi)*latency;
py = py + v*sin(psi)*latency;`

If the delay is not considered, the controller's output will not be based on accurate measurements and the performance will be degraded.
