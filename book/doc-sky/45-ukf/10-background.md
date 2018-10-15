# Background {#ukf-background status=draft}

Before we dive into the UKF, there are some foundations that we should build up:

1. The Bayes Filter

2. Gaussians

3. The Kalman Filter

The basis for the Kalman Filter lies in probability; as such, if you want to better understand some of these probabilistic algorithms, you may find it helpful to brush up on probability. A useful reference on probability and uncertainty is [](#bib:jaynes2003probability).

Since the UKF is an adaptation of the standard Kalman Filter, a lot of our discussion will apply to Kalman Filters in general.

TODO: More comprehensive background section and improved notation coming soon!

### The Bayes Filter

TODO

### Gaussians

TODO: Brief description of Gaussians as needed to discuss the Kalman Filter. Perhaps show multivariate Gaussians as well, e.g., with a 3D plot and an error ellipse to illustrate 2D Gaussian

### The Kalman Filter
#### High-Level Description of the Kalman Filter Algorithm

The Kalman Filter's main goal is to fuse measurement readings with predicted states. For example, if we know (with some degree of uncertainty) that the drone is moving upward, then this knowledge can inform us about the drone's position at the next time step. We can form a **prediction** of the drone's state at the next time step given the drone's current state and any control inputs that we give the drone. Then, at the next time step when we receive a new measurement, we can perform an **update** of our state estimate. In this update step, we look at the difference between the new measurement and the predicted state. This difference is known as the *residual*. Our new state estimate (referred to in literature as the *posterior*) lies somewhere between the predicted state, or *prior*, and the measurement; the scaling factor that accomplishes this is known as the *Kalman gain* [](#bib:labbe_kalman). [](#ukf_predict_update_diagram) depicts this process of prediction and measurement update.

<figure id="ukf_predict_update_diagram">
    <figcaption>Predict-Update Cycle for a One-Dimensional Kalman Filter Tracking Drone Altitude</figcaption>
    <img style='width:30em' src="ukf_predict_update_diagram.png"/>
</figure>

TODO: Fix notation in the diagram

##### State Vector and Covariance Matrix

The KF accomplishes its state estimate by tracking certain **state variables** in a state vector, such as position and velocity along an axis, and the **covariance matrix** corresponding to the state vector. In the first part of this project, your UKF's state vector will track the drone's position and velocity along the $z$-axis and will look like:

$$\mathbf{x}_t=\begin{bmatrix}
z \\
\dot z
\end{bmatrix}$$

where $z$ and $\dot z$ are the position and velocity of the drone along the $z$-axis, respectively. A quick note on symbology: in [](#ukf_predict_update_diagram), we are demonstrating a simple example in which we only track one variable (vertical position), so instead of using boldface vectors, we chose to use scalars. Note that the variable $z$ in [](#ukf_predict_update_diagram) is different than the variable $z$ in this two-dimensional state vector tracking motion along the $z$ axis. In literature, the letter $z$ in some typeface and possibly with a subscript is typically used to denote the measurement (as in [](#ukf_predict_update_diagram)). We will go into more detail about the measurement vector later.

Now, back to our two-dimensional state vector. Why did we decide to track not only position but also velocity? What if we were only interested in the drone's position along the $z$-axis? Well, while in some cases we might only be immediately interested in, say, knowing the robot's position, the addition of $\dot z$ is also important. $\dot z$ is known as a *hidden variable*: we do not have a sensor that allows us to measure this quantity directly. That said, keeping track of this variable allows us to form better estimates about $z$. Why? Since position and velocity are correlated quantities, information about one quantity can inform the other. If the drone has a high positive velocity, for instance, we can be fairly confident that its position at the next time step will be somewhere in the positive direction relative to its previous position. The covariances between position and velocity allow for a reasonable estimate of velocity to be formed---as does any information about acceleration, for example. Chapter 5.7 of Labbe's textbook [](#bib:labbe_kalman) describes the importance of this correlation between state variables such as position and velocity.

The state vector tracks the mean $\mu_t$ of each state variable, which we assume is normally distributed about $\mu_t$. To characterize the uncertainty in this state estimate, we use an $n \times n$ covariance matrix where $n$ is the size of the state vector. For this state vector, then, we define the covariance matrix as:

$$\mathbf{P}_t=\begin{bmatrix}
\sigma^2_z & \sigma_{z,\dot z} \\
\sigma_{\dot z,z} & \sigma^2_{\dot z}
\end{bmatrix}$$

where $\sigma^2_z = \text{Var}\left( z \right)$, for example, denotes the variance in the position estimates and $\sigma_{\dot z,z} = \sigma_{z,\dot z} = \text{Cov}\left( z, \dot z \right)$ denotes the covariance between the position and velocity estimates. As mentioned above, position and velocity are typically positively correlated, as a positive velocity indicates that the drone will likely be at a more positive position at the next time step.

The first frame of [](#ukf_predict_update_diagram) illustrates a state estimate and the standard deviation of that height estimate.

##### State Transition Model for the Prediction Step

The part of the KF that computes a predicted state $\mathbf{\bar x}_t$ is known as the state transition function. The prediction step of the UKF uses the state transition function to propagate the current state at time $t-\Delta t$ to a prediction of the state at the next time step, at time $t$. In standard Kalman Filter literature for linear systems, this transition function can be expressed with two matrices: a state transition matrix $\mathbf{A}_t$ and a control function $\mathbf{B}_t$ that, when multiplied with the current state vector $\mathbf{x}_{t-\Delta t}$ and with the control input vector $\mathbf{u}_t$, respectively, sum together to output the prediction of the next state.

$$
\mathbf{\bar x}_t = \mathbf{A}_t \mathbf{x}_{t-\Delta t} + \mathbf{B}_t \mathbf{u}_t
$$

We give $\mathbf{A}_t$ and $\mathbf{B}_t$ each a subscript $t$ to indicate that these matrices can vary with time. Often, these matrices will include one or more $\Delta t$ terms in order to properly propagate the state estimate forward in time by that amount. If our control input, for example, comes in at a varying frequency, then the time step $\Delta t$ will change.

More generally, in nonlinear systems---where the UKF is useful, which we will describe later---a single transition function $g(\mathbf{x}_{t-\Delta t}, \mathbf{u}_t, \Delta t)$ can express the prediction of what the next state will be given the current state estimate $\mathbf{x}_{t-\Delta t}$, the control input $\mathbf{u}_t$, and the time step $\Delta t$ [](#bib:tellex). For robotic systems such as the PiDrone, the state transition function often involves using kinematic equations to form numerical approximations of the robot's motion in space.

$$
\mathbf{\bar x}_t = g(\mathbf{x}_{t-\Delta t}, \mathbf{u}_t, \Delta t)
$$

The control input that you will use for this project is the linear acceleration along the $z$-axis $\ddot z$ being output by the IMU. While the distinction between this control input and other measurements might seem vague, we can think of these acceleration values as being commands that we set when we control the drone. Indeed, since we control the drone's throttle and thus the downward force of the propellers, we do control the drone's acceleration by Newton's Second Law of Motion. That said, even though in practice people do successfully use IMU accelerations as control inputs, research [](#bib:imu_predict_vs_update) indicates that in certain cases it may be better to use IMU data in the measurement update step; this is an example of a design decision whose performance may depend on the system you are modeling. We choose to use the IMU's acceleration as the control input $\mathbf{u}_t$:

$$
\mathbf{u}_t = \begin{bmatrix} \ddot z \end{bmatrix}
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

As depicted in the third frame of [](#ukf_predict_update_diagram), part of the measurement update step is the computation of the residual $\mathbf{y}_t$. This value is the difference between the measurement and the predicted state. However, the measurement value lives in *measurement space*, while the predicted state lives in *state space*. For this particular 1D example, the measurement and the position estimate represent the same quantity; however, in more complicated systems such as the later part of this project in which you will be implementing a UKF to track multiple spatial dimensions, you will find that the correspondence between measurement and state may require trigonometry. Also, since the sensor measurement often only provides information pertaining to part of the state vector, we cannot always transform a measurement into state space. Chapter 6.6.1 of Labbe's textbook [](#bib:labbe_kalman) describes the distinction between measurement space and state space.

Consequently, we must define a measurement function $h(\mathbf{\bar x}_t)$ that transforms the prior state estimate into measurement space. (For the linear Kalman Filter, this measurement function can be expressed as a matrix $\mathbf{H}_t$.) This transformation allows us to compute the residual in measurement space with the following equation:

$$
\mathbf{y}_t = \mathbf{z}_t - h(\mathbf{\bar x}_t)
$$

Once the residual is computed, the posterior state estimate is computed via the following equation:

$$
\mathbf{x}_t = \mathbf{\bar x}_t + \mathbf{K}_t\mathbf{y}_t
$$

where $\mathbf{K}_t$ is the Kalman gain that scales how much we "trust" the measurement versus the prediction. Once this measurement-updated state estimate $\mathbf{x}_t$ is calculated, the filter continues onto the next predict-update cycle.

The fourth frame of [](#ukf_predict_update_diagram) illustrates this fusion of prediction and measurement in which a point along the residual is selected for the new state estimate by the Kalman gain. The Kalman gain is determined mathematically by taking into account the covariance matrices of the motion model and of the measurement vector. While we do not expect you to know exactly how to compute the Kalman gain, intuitively it is representative of a ratio between the uncertainty in the prior and the uncertainty in the newly measured value.

---

TODO: Write the Bayes Filter algorithm above so that readers can compare with KF algorithm here.

At a high level, that's the Kalman Filter algorithm! Below is the general linear Kalman Filter algorithm [](#bib:tellex) [](#bib:probabilistic_robotics) [](#bib:labbe_kalman) written out in pseudocode. We include $\Delta t$ as an argument to the $\text{predict}()$ function since it so often is used there. We use boldface vectors and matrices to describe this algorithm for the more general multivariate case in which we are tracking more than one state variable, we have more than one measurement variable, et cetera. We also have not yet introduced you to the $\mathbf{Q}_t$ and $\mathbf{R}_t$ matrices yet; you will learn about them later in this project when you implement your first filter. Also, we did not previously mention some of the equations written out in this algorithm (e.g., the computation of the Kalman gain); fret not, however, as you are not responsible for understanding all of the mathematical details. Nonetheless, we give you this algorithm for reference and for completeness. You might also find it helpful to compare the KF algorithm to the Bayes Filter algorithm written above.

&nbsp;

$\hspace{5mm} \textbf{function}\text{ predict}( \mathbf{x}_{t-\Delta t},
    \mathbf{P}_{t-\Delta t}, \mathbf{u}_t, \Delta t )$  
$\hspace{10mm} \texttt{// Compute predicted mean}$  
$\hspace{10mm} \mathbf{\bar x}_t = \mathbf{A}_t \mathbf{x}_{t-\Delta t} + 
    \mathbf{B}_t \mathbf{u}_t$  
$\hspace{10mm} \texttt{// Compute predicted covariance matrix}$  
$\hspace{10mm} \mathbf{\bar P}_t = \mathbf{A}_t \mathbf{P}_{t-\Delta t} \mathbf{A}_t^\mathsf{T} + 
    \mathbf{Q}_t$  
$\hspace{10mm} \text{return } \mathbf{\bar x}_t, \mathbf{\bar P}_t$  

&nbsp;

$\hspace{5mm} \textbf{function}\text{ update}(\mathbf{\bar x}_t,
    \mathbf{\bar P}_t, \mathbf{z}_t)$  
$\hspace{10mm} \texttt{// Compute the residual in measurement space}$  
$\hspace{10mm} \mathbf{y}_t = \mathbf{z}_t - \mathbf{H}_t \mathbf{\bar x}_t$  
$\hspace{10mm} \texttt{// Compute the Kalman gain}$  
$\hspace{10mm} \mathbf{K}_t = \mathbf{\bar P}_t \mathbf{H}_t^\mathsf{T} \left( 
    \mathbf{H}_t \mathbf{\bar P}_t \mathbf{H}_t^\mathsf{T} + \mathbf{R}_t \right)^{-1}$  
$\hspace{10mm} \texttt{// Compute the mean of the posterior state estimate}$  
$\hspace{10mm} \mathbf{x}_t = \mathbf{\bar x}_t + \mathbf{K}_t\mathbf{y}_t$  
$\hspace{10mm} \texttt{// Compute the covariance of the posterior state estimate}$  
$\hspace{10mm} \mathbf{P}_t = \left( \mathbf{I} - \mathbf{K}_t \mathbf{H}_t \right) \mathbf{\bar P}_t$  
$\hspace{10mm} \text{return } \mathbf{x}_t, \mathbf{P}_t$  

&nbsp;

$\hspace{5mm} \textbf{function}\text{ kalman_filter}( \mathbf{x}_{t-\Delta t},
    \mathbf{P}_{t-\Delta t} )$  
$\hspace{10mm} \mathbf{u}_t = \text{get_control_input}()$  
$\hspace{10mm} \Delta t = \text{compute_time_step}()$  
$\hspace{10mm} \mathbf{\bar x}_t, \mathbf{\bar P}_t = \text{predict}(
    \mathbf{x}_{t-\Delta t}, \mathbf{P}_{t-\Delta t}, \mathbf{u}_t, \Delta t )$  
$\hspace{10mm} \mathbf{z}_t = \text{get_sensor_data}()$  
$\hspace{10mm} \mathbf{x}_t, \mathbf{P}_t = \text{update}(\mathbf{\bar x}_t,
    \mathbf{\bar P}_t, \mathbf{z}_t)$  
$\hspace{10mm} \text{return } \mathbf{x}_t, \mathbf{P}_t$  
