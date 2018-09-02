# Background {#ukf-background status=draft}

Before we dive into the UKF, there are some foundations that we should build up:

1. The Bayes Filter

2. Gaussians

3. The Kalman Filter

Since the UKF is an adaptation of the standard Kalman Filter, a lot of our discussion will apply to Kalman Filters in general.

### The Bayes Filter

TODO: Use some of Luke's description

### Gaussians

TODO: Brief description of Gaussians as needed to discuss the Kalman Filter. Perhaps show multivariate Gaussians as well, e.g., with a 3D plot and an error ellipse to illustrate 2D Gaussian

### The Kalman Filter
#### High-Level Description of the Kalman Filter Algorithm

The Kalman Filter's main goal is to fuse measurement readings with predicted states. For example, if we know (with some degree of uncertainty) that the drone is moving upward, then this knowledge can inform us about the drone's position at the next time step. We can form a **prediction** of the drone's state at the next time step given the drone's current state and any control inputs that we give the drone. Then, at the next time step when we receive a new measurement, we can perform an **update** of our state estimate. In this update step, we look at the difference between the new measurement and the predicted state. This difference is known as the *residual*. Our new state estimate (referred to in literature as the *posterior*) lies somewhere between the predicted state, or *prior*, and the measurement; the scaling factor that accomplishes this is known as the *Kalman gain* [](#bib:labbe_kalman). [](#ukf_predict_update_diagram) depicts this process of prediction and measurement update.

<figure id="ukf_predict_update_diagram">
    <figcaption>Predict-Update Cycle for a One-Dimensional Kalman Filter Tracking Drone Altitude</figcaption>
    <img style='width:30em' src="ukf_predict_update_diagram.png"/>
</figure>

##### State Vector and Covariance Matrix

The KF accomplishes its state estimate by tracking certain **state variables** in a state vector, such as position and velocity along an axis, and the **covariance matrix** corresponding to the state vector. In the first part of this project, your UKF's state vector will track the drone's position and velocity along the z-axis and will look like:

$$\mathbf{x}=\begin{bmatrix}
z \\
\dot z
\end{bmatrix}$$

where $z$ and $\dot z$ are the position and velocity of the drone along the z-axis, respectively. A quick note on symbology: in [](#ukf_predict_update_diagram), we are demonstrating a simple example in which we only track one variable (vertical position), so instead of using boldface vectors, we chose to use scalars. Note that the variable $z$ in [](#ukf_predict_update_diagram) is different than the variable $z$ in this two-dimensional state vector tracking motion along the $z$ axis. In literature, the letter $z$ in some typeface and possibly with a subscript is typically used to denote the measurement (as in [](#ukf_predict_update_diagram)). We will go into more detail about the measurement vector later.

Now, back to our two-dimensional state vector. The state vector tracks the mean $\mu$ of each state variable, which we assume is normally distributed about $\mu$. To characterize the uncertainty in this state estimate, we use an $n \times n$ covariance matrix where $n$ is the size of the state vector. For this state vector, then, we define the covariance matrix as:

$$\mathbf{P}=\begin{bmatrix}
\sigma^2_z & \sigma_{z,\dot z} \\
\sigma_{\dot z,z} & \sigma^2_{\dot z}
\end{bmatrix}$$

where $\sigma^2_z = \text{Var}\left( z \right)$, for example, denotes the variance in the position estimates and $\sigma_{\dot z,z} = \sigma_{z,\dot z} = \text{Cov}\left( z, \dot z \right)$ denotes the covariance between the position and velocity estimates. Typically, position and velocity are positively correlated, as a positive velocity indicates that the drone will likely be at a more positive position at the next time step.

The first frame of [](#ukf_predict_update_diagram) illustrates a state estimate and the standard deviation of that height estimate.

##### State Transition Model for the Prediction Step

The part of the KF that computes a predicted state $\mathbf{\bar x}$ is known as the state transition function. The prediction step of the UKF uses the state transition function to propagate the current state to a prediction of the state at the next time step. In standard Kalman Filter literature for linear systems, this transition function can be expressed with two matrices: a state transition matrix $\mathbf{A}$ and a control function $\mathbf{B}$ that, when multiplied with the current state vector $\mathbf{x}$ and with the control input vector $\mathbf{u}$, respectively, sum together to output the prediction of the next state.

More generally, in nonlinear systems---where the UKF is useful, which we will describe later---a single transition function $g(\mathbf{x}, \mathbf{u}, \Delta t)$ can express the prediction of what the next state will be given the current state estimate $\mathbf{x}$, the control input $\mathbf{u}$, and the time step $\Delta t$. [_TODO: Cite Tellex et al. from the Estimation for Quadrotors paper? (should be published on arXiv soon)_] For robotic systems such as the PiDrone, the state transition function often involves using kinematic equations to form numerical approximations of the robot's motion in space.

$$
\mathbf{\bar x} = g(\mathbf{x}, \mathbf{u}, \Delta t)
$$

The control input that you will use for this project is the linear acceleration along the z-axis $\ddot z$ being output by the IMU. While the distinction between this control input and other measurements might seem vague, we can think of these acceleration values as being commands that we set when we control the drone. Indeed, since we control the drone's throttle and thus the downward force of the propellers, we do control the drone's acceleration by Newton's Second Law of Motion.

$$
\mathbf{u} = \begin{bmatrix} \ddot z \end{bmatrix}
$$

The second frame of [](#ukf_predict_update_diagram) shows the result of the state transition function: the drone's state estimate has been propagated forward in time, and in doing so, the uncertainty in its state has increased, since its motion model has some degree of uncertainty and the new measurement has not yet been incorporated.

##### Measurement Function

After the prediction step of the KF comes the measurement update step. When the drone gets a new measurement from one of its sensors, it should compute a new state estimate based on the prediction and the measurement. In the 1D motion model for the first part of this project, the sensor we consider is the infrared (IR) range sensor. We assume that the drone has no roll and pitch, which means that the IR reading directly corresponds to the drone's altitude. The measurement vector, then, is defined as:

$$
\mathbf{z}_t = \begin{bmatrix}
r
\end{bmatrix}
$$

where $r$ is the IR range reading.

As depicted in the third frame of [](#ukf_predict_update_diagram), part of the measurement update step is the computation of the residual $\mathbf{y}$. This value is the difference between the measurement and the predicted state. However, the measurement value lives in *measurement space*, while the predicted state lives in *state space*. For this particular 1D example, the measurement and the position estimate represent the same quantity; however, in more complicated systems such as the later part of this project in which you will be implementing a UKF to track multiple spatial dimensions, you will find that the correspondence between measurement and state may require trigonometry. Also, since the sensor measurement often only provides information pertaining to part of the state vector, we cannot always transform a measurement into state space. Chapter 6.6.1 of Labbe's textbook [](#bib:labbe_kalman) describes the distinction between measurement space and state space.

Consequently, we must define a measurement function $h(\mathbf{\bar x})$ that transforms the prior state estimate into measurement space. This transformation allows us to compute the residual in measurement space with the following equation:

$$
\mathbf{y} = \mathbf{z}_t - h(\mathbf{\bar x})
$$

Once the residual is computed, the posterior state estimate is computed via the following equation:

$$
\mathbf{x} = \mathbf{\bar x} + \mathbf{Ky}
$$

where $\mathbf{K}$ is the Kalman gain that scales how much we "trust" the measurement versus the prediction. Once this measurement-updated state estimate $\mathbf{x}$ is calculated, the filter continues onto the next predict-update cycle.

The fourth frame of [](#ukf_predict_update_diagram) illustrates this fusion of prediction and measurement in which a point along the residual is selected for the new state estimate by the Kalman gain. The Kalman gain is determined mathematically by taking into account the covariance matrices of the motion model and of the measurement vector. While we do not expect you to know exactly how to compute the Kalman gain, intuitively it is representative of a ratio between the uncertainty in the prior and the uncertainty in the newly measured value.

---

At a high level, that's the Kalman Filter algorithm!

TODO: Write out the recursive algorithm. Probably refer to and cite *Probabilistic Robotics* and/or Tellex et al. from the Estimation for Quadrotors paper (should be published on arXiv soon). Perhaps compare to Bayes Filter algorithm, too.