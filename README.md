# Pedestrian-Tracking-Using-Kalman-Filter
The Kalman filter is an algorithm used for state estimation and predicting the new state based on previous measurements. The Kalman filter is widely used in various applications, including object tracking.

## Kalman Filter
### Overview
The Kalman filter is an implementation of a Bayesian filter that operates through an iterative process based on Gaussian distributions. In this filter, the mean of the Gaussian distribution represents the state of the system, which is the primary focus. The variance of the distribution represents the uncertainty associated with the system. Higher variance indicates higher uncertainty in the predictions made by the filter.

The Kalman filter consists of two main phases: prediction and correction. During the prediction phase, the filter estimates the current state based on the previous state and the system dynamics. It uses this estimation to predict the state of the system at the next time step.

In the correction phase, when a measurement is obtained from a sensor or a model, the Kalman filter incorporates this measurement to refine and improve the predictions. The correction step adjusts the predicted state based on how much trust the model places in the measurement compared to the prediction.

By combining the prediction and correction phases, the Kalman filter continuously updates and refines its state estimation, taking into account both the system dynamics and the available measurements.

### Intuition about Kalman equations
![image](https://github.com/TmohamedashrafT/Pedestrian-Tracking-Using-Kalman-Filter/blob/main/kalman%20pipline.png)

Let's find out what these variables mean<br />
X: State matrix. It represents the current state of the system being estimated.<br />
A: State-transition matrix. It describes how the state evolves over time.<br />
B: Control matrix. It relates the control input u to the state transition.<br />
u: Control input. It represents external factors or inputs influencing the state transition.<br />
P: Covariance matrix. It represents the uncertainty or error in the state estimation.<br />
Q: Process error or covariance matrix. It represents the uncertainty in the process or system dynamics.<br />
K: Kalman gain. It is the weight or blending factor applied to the measurement update.<br />
H: Measurement matrix. It relates the state to the measurements obtained from sensors or models.<br />
R: Measurement error or covariance matrix. It represents the uncertainty or error in the measurements.<br />
Z: Measurement It represents the actual measurement.<br />

Let's consider an example to understand how the Kalman equations work. Imagine we have an object whose position we need to predict. We rely on a sensor for measurements, but sensors are never 100% accurate. Therefore, to determine the actual position of the object based on these measurements and the uncertainty of the sensor, we employ a Kalman filter.

Suppose the object is initially at position w_0 with velocity V_0 at time t_0 . We initialize a Kalman filter object using these measurements. During initialization, we set up various matrices. For instance, the covariance matrix initially has a shape of (state size, state size). If we aim to determine the x and y positions and velocities in those directions, the state vector the state vector X would consist of 4 variables, and the covariance matrix would be 4x4 This covariance matrix holds the variances of all variables (uncertainty) and the covariances between these variables. Initially, we assume there's no covariance between any variables.

We also initialize other matrices such as B, which is related to other inputs that might affect velocity (e.g., braking), the process covariance Q related to the uncertainty of the estimator's prediction (shape: 4×4), the measurement error covariance R related to the sensor's error (values in a diagonal matrix of shape: (measurement size,measurement size)), and finally, the measurement matrix  H is responsible for converting the format of P and X to the desired matrix based on what the sensor measures and what we predict from this model (shape:(measurement size,measurement size)).

At t_1, after initializing these matrices and the object, we need to estimate the current state. This involves the prediction phase.in General, If we need to determine the object's position using motion equations, we would use the equation X1 = X0 + V_0 * t + 0.5 * a * t ** 2. In the first prediction equation (Assuming constant velocity (no acceleration)) X_t-1 = A * X_t
with A look like:

      [ 1  0  t  0 ]
      [ 0  1  0  t ]
      [ 0  0  1  0 ]
      [ 0  0  0  1 ]

 

