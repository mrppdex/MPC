
# Model Predictive Control (P4)

## The Model.

This project uses dynamic optimalization technique (ipopt) to minimize the desired loss function in order to compute actuators values passed to the car's simulator..

Loss function penalizes:
- cte error
- epsi errror
- sharp turns of the steering wheel ($$\delta$$)
- too dynamic drive (acceleration)
- temporal changes in an angle of the steering wheel and acceleration
- cte error over the whole horizon distance (by integrating the absolute value of the difference between the desired waypoints and calculated path, using composite Simpson's 3/8 rule)

The state passed to the ipopt is:
- car's current *x* and *y* positions in car's coordinates (0)
- car's current yaw ($$\psi$$) (0)
- current car's speed
- Cross Track Error calculated at the car's current position (x=0)
- EPSI Error calculated at the car's current position ($$\psi$$=0)

The actuator are:
- $$\delta$$ - the angle at which the steering wheel is turned. $$\delta \epsilon[-25^o, 25^o]$$ normalized to $$[-1, 1]$$
- a - car's acceleration $$\epsilon[-1, 1]$$, where a < 0 is breaking and a > 0 accelerating.

Equations used to update the state:
- $$x_{t+1} = x_t + v_t\cos(\psi_t)\Delta{t}$$
- $$y_{t+1} = y_t + v_t\sin(\psi_t)\Delta{t}$$
- $$\psi_{t+1} = \psi_t - v_t\frac{\Delta{t}}{Lf}$$
- $$v_{t+1} = v_t + a_t\Delta{t}$$
- $$cte_{t+1} = (y(0)-f(0)) - (v_t\sin(\psi_t)\Delta{t})$$
- $$e\psi_{t+1} = (y'(0) - \psi(0)) - v_t\frac{\delta_t}{Lf}\Delta{t}$$

## Timestep Length and Elapsed Duration (N & dt).

Hyperparameters:
- N=10. Larger values lead to instability, especially for higher speeds, lower values prevented the cadr from converging to the desired path.
- dt=0.1. 100ms is also the latency. It was important to keep them equal, because MPC::Solve function returns t+1 step to compensate for this latency.

## Polynomial Fitting and MPC Preprocessing.

Car coordinates are passed in as the global map coordinates. They have to be translated to the car coordinates to firstly make it possible to interpolate the waypoints with a polynomial, secondly this is the format of coordinates that simulator uses. Polynomial of the 3rd degree is used, as lower degree polynomial tends to lead to instabilities and higher degree polynomials tend to overfit the data with too "wavey" function.

## Model Predictive Control with Latency.

To deal with the latency, MPC::Solve returns $$\delta_{t+1}$$ and $$a_{t+1}$$ instead of  $$\delta_{t}$$ and $$a_{t}$$. It is important to keep dt equal to the latency.

## Running the code

Refer to Original Project's git: https://github.com/udacity/CarND-MPC-Project
