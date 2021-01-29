# KalmanFilterCode
This matlab file models the two-dimensional movement of an airplane. The simulation shows how a Kalman filter can be used to overcome noise introduced to the controls. Four sections outline the code:

1. Implement airplane dynamics using a linear system, suitable for use in a Kalman filter - and implenment a simple policy
2. Add Gaussian noise to the control and plot trajectory
3. Create a noisy observation function by adding Gaussian noise to the airplane position and velocity
4. Implement a Kalman filter and show the estimated trajectory of the plane overlaid on its true trajectory (in addition to the noisy observation)

