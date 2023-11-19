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
with State transition (A):

      [ 1  0  t  0 ]
      [ 0  1  0  t ]
      [ 0  0  1  0 ]
      [ 0  0  0  1 ]
And State matrix (X):

      [ x   ]
      [ y   ] 
      [ x_v ]
      [ y_v ]
 
So, the result will be:

      [ x + x_v * t ]
      [ y + y_v * t ]
      [     x_v     ]
      [     y_v     ]
This represents the new state. Moving to the second equation in the prediction phase to predict the covariance matrix, A.P.A.T updates and alters the entire covariance matrix. 
The Covariance matrix (P):

      [ var(x) 0 0  0  ]
      [ 0 var(y) 0  0  ]
      [ 0 0 var(x_v) 0 ]
      [ 0 0 0 var(y_v] ]
the result will be:

      [ var(x) + var(x_v) * t **2       0                         var(x_v) * t                   0      ]
      [ 0                         var(y) + var(y_v) * t **2            0                  var(y_v) * t  ]
      [ var(x_v) * t                     0                          var(x_v)                   0        ]
      [ 0                             var(y_v) * t                     0                   var(y_v)     ]
This new matrix shows the covariance between the positions and their velocities, while there is no covariance between two positions or two velocities, which makes sense.

Now, let's move on to the update phase, also referred to as the correction phase. After calculating the predictions, we receive measurements from the sensor. Then, we compare the prediction with the measurement by applying the following equations.

In the initial equations, we calculate the Kalman gain. The intuition behind the Kalman gain lies in providing a number within the range of 0 to 1. This number indicates whether the higher error exists in the estimator or in the measurement. For instance, if the error in the estimator is considerably high compared to the relatively small measurement error, the Kalman gain will tend to converge towards one, and vice versa.

![image](https://github.com/TmohamedashrafT/Pedestrian-Tracking-Using-Kalman-Filter/blob/main/Kalman%20gain%20eq.webp)

After calculating the Kalman gain, the next step involves updating the state matrix (X) and the covariance matrix (P). To update X, we compute the difference between the measurement and the predicted value. Then, we multiply this result by the Kalman gain and add it to the predicted state.

The rationale behind this step is to guide the estimator in placing trust either in the prediction or in the measurement. For instance, if the Kalman gain is closer to one, indicating that the measurement error is significantly lower compared to the prediction error, the estimator should place more emphasis on the measurement. Consequently, the disparity between the measurement and the prediction will have a greater impact on the predicted value.

Conversely, when the Kalman gain is lower, signaling that the prediction error is considerably lower than the measurement error, the estimator diminishes the influence of this difference. In such cases, the estimator places greater confidence in the predicted value.


Finally, in the iteration update, the covariance matrix is adjusted based on the error in the measurement. If the measurement error is high, the model needs to be cautious about trusting the data. Consequently, it shouldn't rapidly diminish the prediction error. Therefore, the estimator reduces the error incrementally.

Conversely, when the measurement error is very low, the estimator relies heavily on the measurement, leading to quicker convergence to the real state and a reduction in prediction error.

This behavior is reflected in the equation that computes the difference between the identity matrix and the Kalman gain, which is then multiplied by the covariance matrix. If the Kalman gain is high (indicating low measurement error), the term (I - KG) will be very small. As a result, it rapidly diminishes the error in the covariance. Conversely, if the Kalman gain is low, the adjustment will be slower.

## Tracking using Kalman filter 
In a tracking problem pipeline, we only receive the position from the detector model. Therefore, we initialize the velocity as zeros. The state vector X will consist of (x_min, y_min, x_max, y_max, x_min_velocity, y_min_velocity, x_max_velocity, y_max_velocity), while assigning high uncertainty to the velocity components. Additionally, the Measurement matrix will be

      [ 1 0 0 0 0 0 0 0 ]
      [ 0 1 0 0 0 0 0 0 ]
      [ 0 0 1 0 0 0 0 0 ]
      [ 0 0 0 1 0 0 0 0 ]

since we receive only 4 measurements from the model.The estimator calculates the velocity of points when a new position measurement becomes available from the model. The Kalman filter compares this measured position with the predicted position obtained from the previous step. By calculating the difference between these positions, the filter estimates how much the object's actual position differs from the predicted position.

The Kalman filter is implemented in this file [`Kalman_filter`](https://github.com/TmohamedashrafT/Pedestrian-Tracking-Using-Kalman-Filter/blob/main/Kalman_filter.py).

The estimator was experimented in pedestrian tracking problems, using the Hungarian algorithm as the association metric and YOLOv5s as the detector. The results are as follows
[<img src="https://github.com/TmohamedashrafT/Pedestrian-Tracking-Using-Kalman-Filter/blob/main/kalman%20pipline.png" width="600" height="300"
/>](https://github.com/TmohamedashrafT/Pedestrian-Tracking-Using-Kalman-Filter/blob/main/output.mp4)


















