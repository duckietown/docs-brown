# Project {#ukf-project status=draft}

## Background
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

#### A Closer Look at Some of the Math of the Kalman Filter

TODO: If this section seems necessary, then figure out what to put here

---

## The Unscented Kalman Filter: Nonlinear State Estimation

### Limitations of the Standard (Linear) Kalman Filter

So far, we have discussed the standard Kalman Filter algorithm. However, we have not mentioned its limitations. The standard Kalman Filter assumes that the system is both *linear* and *Gaussian*. In other words, the uncertainties in the motion and measurement models are assumed to be normally distributed about a mean in order to produce optimal estimates, which allows us to represent the state estimate as a Gaussian with mean and variance. For many systems, the Gaussian assumption is a good one. Intuitively, one can imagine that a sensor's noise, for example, varies more or less symmetrically about a true mean value, with larger deviations occurring less frequently.

The greater constraint, however, is the assumption that the system is linear. What we mean by this is that the state transition function and measurement function are linear functions, and as a result, when we pass Gaussian distributions through these functions, the output remains Gaussian or proportional to a Gaussian. An arbitrary nonlinear function, on the other hand, will not output another Gaussian or scaled Gaussian, which is a problem since so much of the Kalman Filter math depends on the state estimate being Gaussian. The Unscented Kalman Filter was expressly designed to robustly handle this issue of nonlinearity.

In this project's z-axis UKF, the functions are linear, so indeed a standard Kalman Filter would suffice. However, for the second UKF that you will be implementing, there are nonlinearities due to the drone's orientation in space. To make the transition easier from the first part to the second part of this project, we are asking you to implement a UKF even for a linear system. The UKF estimates will be the same as a KF; the only downsides might be code complexity and computation time. That said, you will be using a Python library called FilterPy (written by Labbe, author of *Kalman and Bayesian Filters in Python* [](#bib:labbe_kalman)) that handles and hides most of the filtering math anyway.

You might also be wondering what the term "unscented" has to do with a Kalman Filter that applies to nonlinear systems. There is no greater technical meaning to the word; the inventor claims it is an arbitrary choice that resulted from his catching a glimpse of a coworker's deodorant while trying to come up with a name for his filter [](#bib:uhlmann).

### Underlying Principle of the UKF

To handle the nonlinearities, the UKF uses a sampling scheme. An alternative to the UKF known as the Extended Kalman Filter (EKF) uses Jacobians to linearize the nonlinear equations, but the UKF takes a deterministic sampling approach that in many cases results in more accurate estimates and is a simpler algorithm to implement [_TODO: Cite Tellex et al. from the Estimation for Quadrotors paper (should be published on arXiv soon)_].

The UKF uses a function to compute so-called **sigma points**, which are the sample points to pass through the state transition and measurement functions. Each sigma point also has corresponding **weights** for the sample's mean and covariance. The sigma points are generated so that there are $2n+1$ of them, where $n$ is the size of the state vector. Imagine a one-dimensional state vector, for example, which we represent as a single-variable Gaussian. In this instance, $2(1)+1=3$ sigma points are chosen. One of these points is the mean of the Gaussian, and the two other points are symmetric about the mean on either side. The exact distance of these points from the mean sigma point will vary depending on parameters passed into the sigma point function, but we do not expect you to worry about these parameters. The idea, though, is that these $2(1)+1=3$ sigma points and their weights are sufficiently representative of the Gaussian distribution.

The idea, then, is that these points that represent the Gaussian state estimate can be passed through a nonlinear function (i.e., the state transition or measurement functions), which can scatter the points arbitrarily. We then want to recover a Gaussian from these scattered points, and we do so by using the **unscented transform**, which computes a new mean and covariance matrix. To compute the new mean, the unscented transform calculates a weighted sum of each sigma point with its associate sample mean weight.

### UKF in the Prediction Step

The UKF formulates the prior state estimate by specifying a set of sigma points $\boldsymbol{\mathcal{X}}$ according to the current state estimate and then propagating these points through the state transition function to yield a new set of sigma points $\boldsymbol{\mathcal{Y}}$, which are passed through the unscented transform to produce the prior state estimate.

### UKF in the Update Step

For the update step, we pass the prior sigma points $\boldsymbol{\mathcal{Y}}$ through the measurement function to yield a set of measurement sigma points $\boldsymbol{\mathcal{Z}} = h(\boldsymbol{\mathcal{Y}})$. This set of points is then passed through the unscented transform, which computes the mean and covariance of these sample points. The mean is subtracted from the measurement to compute the residual. Labbe chapter 10.5.2 [](#bib:labbe_kalman) goes into more detail about this computation, as well as describing the computation of the Kalman gain specific to the UKF. Once we have a prior state estimate, a residual, and a Kalman gain, we can compute the posterior state estimate as usual with $\mathbf{x} = \mathbf{\bar x} + \mathbf{Ky}$.

---

## Steps to Design and Implement a Kalman Filter on a Robot

To apply a Kalman Filter (linear KF or UKF) to a specific robot, there are certain parts of the algorithm that we need to define.

1. **State Vector:** The first aspect of the KF design process specific to the robot application is the selection of state variables to track in the state vector.
2. **Motion Model:** The motion model of the robot demands careful thought when designing a KF, as it determines the state transition function.
3. **Measurement Model:** The robot's sensor suite plays a significant role in how the robot forms state estimates. Its sensors determine the measurement function.
4. **Process Noise and Measurement Covariance Matrices:** The process noise and measurement covariance matrices must be determined from the motion model and sensor suite, respectively.
5. **Initialization of the Filter:** The filter must have initial values on which to perform the predictions and measurement updates.
6. **Asynchronous Inputs:** Sometimes, the KF has to be adapted to handle asynchronous inputs from real-world sensors, whose data rates are not strictly fixed.
7. **Tuning and Testing:** Finally, once a filter is implemented, it is a good idea to tune and test it in simulation and then on the real robot, quantifying its performance if possible.

We will be going over these design decisions and implementation details step-by-step as you implement your filters in one and three spatial dimensions on the drone.

---

## Our Implementation: UKF in One Spatial Dimension
Finally, it is time for you to design and implement a UKF specific to the PiDrone! We glossed over a lot of the mathematical details of the KF and UKF because we think it's more important that you understand the high-level workings of the Kalman Filter. Also, as a roboticist, the more difficult aspect of Kalman filtering is indeed the process of designing a filter for your robot's needs and capabilities. The underlying math mostly stays the same across UKF implementations, but the design (essentially, the seven steps described in the previous section) is tailored to the application.

As a result, we will have you use the Python library FilterPy, which abstracts away most of the nitty-gritty math.

This part of the project has **three deliverables** in the `pidrone_project2_ukf` repository, which are to be accessed and submitted via GitHub Classroom:

1. A $\LaTeX$ document `ukf2d_written_solutions.tex` with the answers to the UKF design and implementation questions.
2. Your implementation of the UKF written in the `StateEstimators/student_state_estimator_ukf_2d.py` stencil code. In this stencil code file, we have placed "TODO" tags describing where you should write your solution code to the relevant problems.
3. A $\LaTeX$ document `ground_robot_ukf.tex` with the solutions to the problem on designing a UKF for an imaginary ground robot system.

TODO: Include better description of how to download the code when it's set up on GitHub Classroom, and also how to submit the solution code

### Design and Implement the Filter

In addition to implementing the UKF in code, we want you to learn about the design process, much of which occurs outside of the code that will run the UKF. Plus, we have some questions we want you to answer in writing to demonstrate your understanding of the UKF. Hence, you will be writing up some of your solutions in $\LaTeX$.

**Task:** From the `pidrone_project2_ukf` repository, open up the `ukf2d_written_solutions.tex` file in your favorite $\LaTeX$ editor. This could be in Overleaf, your Brown CS department account, or locally on your own computer. *Before submitting your document, please make sure your document compiles. If you are having trouble with $\LaTeX$, please seek out the help of a TA.*

### State Vector
For this part of the project, as we have mentioned before in this project description, we are going to track the drone's position and velocity along the z-axis:

$$\mathbf{x}=\begin{bmatrix}
z \\
\dot z
\end{bmatrix}$$

### State Transition Function
**Task:** Implement the state transition function $g(\mathbf{x}, \mathbf{u}, \Delta t)$ by filling in the template given in Section 1.2.2 of `ukf2d_written_solutions.tex` with the correct values to propagate the current state estimate forward in time. Remember that for the drone, this involves kinematics (hint: use the constant acceleration kinematics equations). Since there is a notion of transitioning the state from the previous time step, this function will involve the variable $\Delta t$.

**Task:** Translate the state transition function into Python by filling in the `state_transition_function()` method. Follow the "TODO"s there. Note the function's type signature for the inputs and outputs.

### Measurement Function
**Task:** In Section 1.3.2 of `ukf2d_written_solutions.tex`, implement the measurement function $h(\mathbf{\bar x})$ to transform the prior state estimate into measurement space. For this model's state vector and measurement vector, $h(\mathbf{\bar x})$ can be implemented as a $1 \times 2$ matrix that is multiplied with the $2 \times 1$ state vector, outputting a $1 \times 1$ matrix: the same dimension as the measurement vector $\mathbf{z}_t$, which allows for the computation of the residual.

**Task:** As before, translate the measurement function into code by filling in the `measurement_function()` method. Follow the "TODO"s there. Note the function's type signature for the inputs and outputs.

### Process Noise and Measurement Covariance Matrices
The process noise covariance matrix $\mathbf{Q}$ needs to be determined for the prediction step, but you do not need to determine this yourself, as this matrix can be notoriously difficult to ascertain. Feasible values for the elements of $\mathbf{Q}$ are provided in the code.

On the other hand, the measurement noise covariance matrix $\mathbf{R}$ has a more tangible meaning: it represents the variance in our sensor readings, along with covariances if sensor readings are correlated. For our 1D measurement vector, this matrix just contains the variance of the IR sensor.

**Task:** Characterize the noise in the IR sensor by experimentally collecting data from your drone in a stationary setup and computing its variance. To do so, prop the drone up so that it is stationary and its IR sensor is about 0.3 m from the ground, pointing down, unobstructed. To collect the range data, execute the following commands on your drone:

Navigate to pidrone_pkg and run ROS:

    duckiebot $ roscd pidrone_pkg
    $ screen -c pi.screenrc

After navigating to a free screen, echo the infrared ROS topic and extract just the range value. To automatically log a lot of IR range readings, you must redirect standard output to a file like so:

    duckiebot $ rostopic echo /pidrone/infrared/range > ir_data.txt

We have provided a script `ir_sample_variance_calculation.py` that reads in the range readings from the file (so make sure this file is named `ir_data.txt` and is in the same directory as `ir_sample_variance_calculation.py`), computes the sample variance, and plots the distribution of readings using `matplotlib`. If you want to run this on your drone, then you will have to ensure that your `ssh` client has the capability to view pop-up GUI windows in order to view the plot. If you have XQuartz installed on your base station, for example, then this should allow you to run `ssh -Y pi@defaultdrone` (substituting your drone's hostname for `defaultdrone`, of course). Otherwise, you can run this script on a computer that has Python, `matplotlib`, and `numpy` installed.

Your plot should look somewhat Gaussian, as in [](#ir_sample_distribution).

<figure id="ir_sample_distribution">
    <figcaption>Sample Distribution of Infrared Range Data</figcaption>
    <img style='width:30em' src="ir_sample_distribution.png"/>
</figure>

TODO: Make sure students get FilterPy and matplotlib packages, for example, on their drone (pip install). Perhaps see about getting these dependencies included in the image that students flash on their SD cards.

TODO: Include the `ir_sample_variance_calculation.py` script in the students' GitHub repo. Test that it works. Change ir_test.txt to ir_data.txt.

**Task:** Record the resulting sample variance value in Section 1.3.3 of `ukf2d_written_solutions.tex`.

**Task:** Enter this value into the code for `self.ukf.R` in the `initialize_ukf_matrices()` method.

### Initialize the Filter

Before the UKF can begin its routine of predicting and updating state estimates, it must be initialized with values for the state estimate $\mathbf{x}$ and state covariance matrix $\mathbf{P}$, as the first prediction call will rely on propagating these estimates forward in time. There is no set way to initialize the filter, but one common approach is to simply take the first measurements that the system receives and treat them as the best estimate of the state until we have estimates for each variable.

**Task:** For your drone, you want to wait until the first IR reading comes in and then set the corresponding $z$ position value equal to this measurement. This only accounts for one of the two state variables. For now, initialize $\dot z=0 \text{ m/s}$. Go ahead and implement this state estimate initialization in code in the `ir_data_callback()` method, which gets called each time this ROS node receives a message published by the IR sensor.

**Task:** In addition to initializing the state estimate, you must initialize the time value corresponding to the state estimate. We provide a method `initialize_input_time()` that accomplishes this, but you must call it in the appropriate location.

Another aspect of the filter that can be initialized upon the first receipt of a measurement is the state covariance matrix $\mathbf{P}$. How do we know what values to use for this initialization? Again, this is a design decision that can vary by application. We can directly use the variance of the IR sensor to estimate an initial variance for the height estimate. We won't worry about initializing the velocity variance or the covariances. If we always knew that we were going to start the filter while the drone is at rest, then we could confidently initialize velocity to 0 and assign a low variance to this estimate.

**Task:** Initialize the $\mathbf{P}$ matrix in the `ir_data_callback()` method with the variance of the IR sensor for the variance of the $z$ position estimate.

**Task:** How else could you initialize the estimate for $\dot z$ given the raw range readings from the IR sensor? Describe in `ukf2d_written_solutions.tex` what you would do and the potential pros and cons of your approach. Do not implement this in code.

It is unlikely that the filter initialization will be perfect. Fret not---the Kalman Filter can handle poor initial conditions and eventually still converge to an accurate state estimate. Once your predict-update loop is written, we will be testing out the impact of filter initialization.

### Asynchronous Inputs

The traditional Kalman Filter is described as a loop alternating between predictions and measurement updates. In the real world, however, we might receive control inputs more frequently than we receive measurement updates; as such, instead of throwing away information, we would prefer to perform multiple consecutive predictions. Additionally, our inputs (i.e., control inputs and sensor data) generally arrive asynchronously, yet the traditional Kalman Filter algorithm has the prediction and update steps happen at the same point in time. Furthermore, the sample rates of our inputs are typically not constant, and so we cannot design our filter to be time invariant.

**Task:** Describe why, in a real-world Kalman Filter implementation, it makes sense to be able to perform multiple consecutive predictions before performing a new measurement update, whereas it does not make sense algorithmically to perform multiple consecutive measurement updates before forming a new prediction. It might be helpful to think about the differences between what happens to the state estimate in the prediction versus the update step. Write your answer in `ukf2d_written_solutions.tex`.

**Task:** Implement the predicting and updating of your UKF, keeping in mind the issue of asynchronous inputs. These steps will occur in two ROS subscriber callbacks: 1) `imu_data_callback` when an IMU control input is received and 2) `ir_data_callback` when an IR measurement is received. Remember that we want to perform a prediction not only when we receive a new control input but also when we receive a new measurement in order to propagate the state estimate forward to the time of the measurement. One way to do this prediction without a new control input is to interpolate and assume that the control input remains the same as last time (which is what we suggest); another potential approach might be to not include a control input in those instances (i.e., set it to zeros). The method for our FilterPy UKF object that you want to use to perform the prediction is `self.ukf.predict()`, which takes in a keyword argument `dt` that is the time step since the last state estimate and a keyword argument `u`, corresponding to the argument `u` of `state_transition_function()`, that is a NumPy array with the control input(s). The method to do a measurement update is `self.ukf.update()`, which requires a positional argument consisting of a measurement vector as a NumPy array. Call `self.publish_current_state()` at the end of each callback to publish the new state estimate to a ROS topic.

Note that these callbacks get called in new threads; therefore, there is the potential for collisions when, say, both IMU and IR data come in almost at the same time and one thread has not had the opportunity to finish its UKF computations. We don't want both threads trying to simultaneously alter the values of certain variables, such as the $\mathbf{P}$ matrix when doing a prediction, as this can cause the filter to output nonsensical results and break. Therefore, we have implemented a simple callback blocking scheme---using the `self.in_callback` variable---that ignores a new callback if another callback is going on, essentially dropping packets if there are collisions. While this may not be the optimal or most stable way to handle the issue (one could imagine the IMU callback, for example, always blocking the IR callback and hence preventing measurement updates), it at least gets rid of the errors that would occur with collisions. If you so desire, feel free to implement your own callback management system that perhaps balances the time allocated to different callbacks.

### Tune and Test the Filter
TODO: Write this section and include description on quantitatively evaluating the performance of the filter

**Task:** Describe how a (well-tuned) Kalman Filter outperforms an exponential moving average (EMA) filter applied to raw sensor data. Attach an image of the Height Readings graph showing the difference between your UKF and the EMA, and briefly describe the different features.

**Task:** To test out your UKF's robustness in the face of poor initialization, compare how long it takes the state estimates to converge to accurate values with good initial conditions and with poor initial conditions.
TODO: Look into the best way in which students should be characterizing filter performance. Perhaps by evaluating state estimates against MoCap ground truth and seeing if the filter's covariance matrix $\mathbf{P}$ reports accurate variances. This seems like an intuitive and mathematically rigorous enough method. Or, if students are developing their UKF on simulated data, then ground truth would be the mean values in the simulation about which we are generating noise.

### Exercise: Designing a UKF for a Ground Robot

At this point, you should have a functioning 2D UKF on your drone---congratulations! Before you get checked off for your work thus far, we want to introduce you to designing a slightly more complicated UKF for a different robot system that includes nonlinearities in the state transition and measurement functions. Here is the scenario:

Imagine a ground robot capable of moving in the $xy$-plane. Its body frame is oriented such that the origin is located at its center, the positive $x$-axis points to the right of the robot, the positive $y$-axis points forward, and the positive $z$-axis points up. The robot can control its forward acceleration $\ddot y^b$ in the body frame and its heading $\psi$ (rotation about its center). Its sensor suite consists of a forward-pointing range sensor. With this setup:

- What is a reasonable state vector?
- Define the state transition function (hint: there will be nonlinearities)
- Define the measurement function (hint: there will be nonlinearities)

**Task:** Write up your solutions to the above bullet points in $\LaTeX$ in the file `ground_robot_ukf.tex`.

TODO: Figure out a better way to frame what the state vector should be perhaps. In other words, work through this problem first and then make sure it's reasonable for students to do it.

## Check-Off
Come to TA hours to get checked off when you have fulfilled the following:

- Relevant solutions are written up in `ukf2d_written_solutions.tex`. $\LaTeX$ source is pushed to GitHub.
- UKF is implemented in code in `student_state_estimator_ukf_2d.py` and pushed to GitHub. Your UKF can run:
    1. On simulated data. You can quantify its performance relative to simulated ground truth and see it working better than the EMA
    2. On your drone while manually moving it up and down. You can see in the web interface Height Readings chart that it performs better than the EMA when moving the drone
    3. On your drone while it is in flight, with comparable or better performance than when running the EMA filter for state estimation
- Relevant solutions are written up in `ground_robot_ukf.tex`.

Now that you have derived the mathematical design for your 2D UKF and implemented it, come to TA hours with your $\LaTeX$ document and code pushed to GitHub to get checked off. In addition to these deliverables, we will be asking you to verbally describe some of the answers you wrote up in your $\LaTeX$ document, which will enable us to better assess your understanding of the work in the project so far and your readiness for the next stage of the project. Feel free to also ask the TAs questions about the next part of the project during your check-off.

---

## Our Implementation: UKF in Three Spatial Dimensions
TODO

### State Vector
TODO

### State Transition Function
TODO

### Measurement Function
TODO

### Process Noise and Measurement Covariance Matrices
TODO

### Initialize the Filter
TODO

### Asynchronous Inputs
TODO

### Tune and Test the Filter
TODO: Write this section and include description on quantitatively evaluating the performance of the filter



