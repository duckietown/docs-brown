# Our Implementation: UKF in One Spatial Dimension {#ukf-one-spatial-dimension status=draft}

Finally, it is time for you to design and implement a UKF specific to the PiDrone! We glossed over a lot of the mathematical details of the KF and UKF because we think it's more important that you understand the high-level workings of the Kalman Filter. Also, as a roboticist, the more difficult aspect of Kalman filtering is indeed the process of designing a filter for your robot's needs and capabilities. The underlying math mostly stays the same across UKF implementations, but the design (essentially, the seven steps described in [the previous section](#ukf-design-and-implementation-steps)) is tailored to the application.

As a result, we will have you use the Python library FilterPy, which abstracts away most of the nitty-gritty math.

This part of the project has **three deliverables** in the `pidrone_project2_ukf` repository, which are to be accessed and submitted via GitHub Classroom:

1. A $\LaTeX$ document `ukf2d_written_solutions.tex` with the answers to the UKF design and implementation questions.
2. Your implementation of the UKF written in the `StateEstimators/student_state_estimator_ukf_2d.py` stencil code. In this stencil code file, we have placed "TODO" tags describing where you should write your solution code to the relevant problems.
3. A $\LaTeX$ document `ground_robot_ukf.tex` with the solutions to the problem on designing a UKF for an imaginary ground robot system.

TODO: Include better description of how to download the code when it's set up on GitHub Classroom, and also how to submit the solution code

## Design and Implement the Filter

In addition to implementing the UKF in code, we want you to learn about the design process, much of which occurs outside of the code that will run the UKF. Plus, we have some questions we want you to answer in writing to demonstrate your understanding of the UKF. Hence, you will be writing up some of your solutions in $\LaTeX$.

**Task:** From the `pidrone_project2_ukf` repository, open up the `ukf2d_written_solutions.tex` file in your favorite $\LaTeX$ editor. This could be in Overleaf, your Brown CS department account, or locally on your own computer. *Before submitting your document, please make sure your document compiles. If you are having trouble with $\LaTeX$, please seek out the help of a TA.*

## State Vector
For this part of the project, as we have mentioned before in this project description, we are going to track the drone's position and velocity along the z-axis:

$$\mathbf{x}=\begin{bmatrix}
z \\
\dot z
\end{bmatrix}$$

## State Transition Function
**Task:** Implement the state transition function $g(\mathbf{x}, \mathbf{u}, \Delta t)$ by filling in the template given in Section 1.2.2 of `ukf2d_written_solutions.tex` with the correct values to propagate the current state estimate forward in time. Remember that for the drone, this involves kinematics (hint: use the constant acceleration kinematics equations). Since there is a notion of transitioning the state from the previous time step, this function will involve the variable $\Delta t$.

**Task:** Translate the state transition function into Python by filling in the `state_transition_function()` method in `StateEstimators/student_state_estimator_ukf_2d.py`. Follow the "TODO"s there. Note the function's type signature for the inputs and outputs.

## Measurement Function
**Task:** In Section 1.3.2 of `ukf2d_written_solutions.tex`, implement the measurement function $h(\mathbf{\bar x})$ to transform the prior state estimate into measurement space. For this model's state vector and measurement vector, $h(\mathbf{\bar x})$ can be implemented as a $1 \times 2$ matrix that is multiplied with the $2 \times 1$ state vector, outputting a $1 \times 1$ matrix: the same dimension as the measurement vector $\mathbf{z}_t$, which allows for the computation of the residual.

**Task:** As before, translate the measurement function into code, this time by filling in the `measurement_function()` method. Follow the "TODO"s there. Note the function's type signature for the inputs and outputs.

## Process Noise and Measurement Covariance Matrices
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

When running `ir_sample_variance_calculation.py`, you can pass in command-line arguments of `-l` to plot a line chart instead of a bar chart and `-n` followed by a positive integer to indicate the number of intervals to use for the histogram (defaults to 100 intervals).

**Task:** Record the resulting sample variance value in Section 1.3.3 of `ukf2d_written_solutions.tex`. Also include an image of your histogram in `ukf2d_written_solutions.tex`.

**Task:** Enter this sample variance value into the code for `self.ukf.R` in the `initialize_ukf_matrices()` method.

## Initialize the Filter

Before the UKF can begin its routine of predicting and updating state estimates, it must be initialized with values for the state estimate $\mathbf{x}$ and state covariance matrix $\mathbf{P}$, as the first prediction call will rely on propagating these estimates forward in time. There is no set way to initialize the filter, but one common approach is to simply take the first measurements that the system receives and treat them as the best estimate of the state until we have estimates for each variable.

**Task:** For your drone, you want to wait until the first IR reading comes in and then set the corresponding $z$ position value equal to this measurement. This only accounts for one of the two state variables. For now, initialize $\dot z=0 \text{ m/s}$. Go ahead and implement this state estimate initialization in code in the `ir_data_callback()` method, which gets called each time this ROS node receives a message published by the IR sensor.

**Task:** In addition to initializing the state estimate, you must initialize the time value corresponding to the state estimate. We provide a method `initialize_input_time()` that accomplishes this, but you must call it in the appropriate location.

Another aspect of the filter that can be initialized upon the first receipt of a measurement is the state covariance matrix $\mathbf{P}$. How do we know what values to use for this initialization? Again, this is a design decision that can vary by application. We can directly use the variance of the IR sensor to estimate an initial variance for the height estimate. We won't worry about initializing the velocity variance or the covariances. If we always knew that we were going to start the filter while the drone is at rest, then we could confidently initialize velocity to 0 and assign a low variance to this estimate.

**Task:** Initialize the $\mathbf{P}$ matrix in the `ir_data_callback()` method with the variance of the IR sensor for the variance of the $z$ position estimate.

**Task:** How else could you initialize the estimate for $\dot z$ given the raw range readings from the IR sensor? Describe in `ukf2d_written_solutions.tex` what you would do and the potential pros and cons of your approach. Do not implement this in code.

It is unlikely that the filter initialization will be perfect. Fret not---the Kalman Filter can handle poor initial conditions and eventually still converge to an accurate state estimate. Once your predict-update loop is written, we will be testing out the impact of filter initialization.

## Asynchronous Inputs

The traditional Kalman Filter is described as a loop alternating between predictions and measurement updates. In the real world, however, we might receive control inputs more frequently than we receive measurement updates; as such, instead of throwing away information, we would prefer to perform multiple consecutive predictions. Additionally, our inputs (i.e., control inputs and sensor data) generally arrive asynchronously, yet the traditional Kalman Filter algorithm has the prediction and update steps happen at the same point in time. Furthermore, the sample rates of our inputs are typically not constant, and so we cannot design our filter to be time invariant.

**Task:** Describe why, in a real-world Kalman Filter implementation, it makes sense to be able to perform multiple consecutive predictions before performing a new measurement update, whereas it does not make sense algorithmically to perform multiple consecutive measurement updates before forming a new prediction. It might be helpful to think about the differences between what happens to the state estimate in the prediction versus the update step. Write your answer in `ukf2d_written_solutions.tex`.

**Task:** Implement the predicting and updating of your UKF, keeping in mind the issue of asynchronous inputs. These steps will occur in two ROS subscriber callbacks: 1) `imu_data_callback` when an IMU control input is received and 2) `ir_data_callback` when an IR measurement is received. Remember that we want to perform a prediction not only when we receive a new control input but also when we receive a new measurement in order to propagate the state estimate forward to the time of the measurement. One way to do this prediction without a new control input is to interpolate and assume that the control input remains the same as last time (which is what we suggest); another potential approach might be to not include a control input in those instances (i.e., set it to zeros). The method for our FilterPy UKF object that you want to use to perform the prediction is `self.ukf.predict()`, which takes in a keyword argument `dt` that is the time step since the last state estimate and a keyword argument `u`, corresponding to the argument `u` of `state_transition_function()`, that is a NumPy array with the control input(s). The method to do a measurement update is `self.ukf.update()`, which requires a positional argument consisting of a measurement vector as a NumPy array. Call `self.publish_current_state()` at the end of each callback to publish the new state estimate to a ROS topic.

Note that these callbacks get called in new threads; therefore, there is the potential for collisions when, say, both IMU and IR data come in almost at the same time and one thread has not had the opportunity to finish its UKF computations. We don't want both threads trying to simultaneously alter the values of certain variables, such as the $\mathbf{P}$ matrix when doing a prediction, as this can cause the filter to output nonsensical results and break. Therefore, we have implemented a simple callback blocking scheme---using the `self.in_callback` variable---that ignores a new callback if another callback is going on, essentially dropping packets if there are collisions. While this may not be the optimal or most stable way to handle the issue (one could imagine the IMU callback, for example, always blocking the IR callback and hence preventing measurement updates), it at least gets rid of the errors that would occur with collisions. If you so desire, feel free to implement your own callback management system that perhaps balances the time allocated to different callbacks.

## Tune and Test the Filter
TODO: Write this section and include description on quantitatively evaluating the performance of the filter

In this problem, you will be testing your UKF that you have implemented thus far. You will start by testing on simulated drone data. We have set up the simulation to publish its data on ROS topics so that your UKF program interfaces with the drone’s ROS environment and will be able to be applied directly to real, live data coming from the drone during flight. The output from the UKF can be evaluated in the JavaScript web interface (see pidrone_pkg/web/index.html).

To run your UKF with simulated drone data, run ROS as usual with the `pi.screenrc` file in `pidrone_pkg`. Upon start-up, go ahead and terminate the IR and flight controller nodes, as these would conflict with the drone simulator's simulated sensors. In a free screen, navigate to the `pidrone_project2_ukf` repository and run the following command:

    duckiebot $ python state_estimator.py --primary ukf2d --others simulator

This command will automatically run your 2D UKF as the primary state estimator, along with the drone simulator. The EMA filter will also be run automatically with the 2D UKF, since the 2D UKF does not provide a very complete state vector in three-dimensional flight scenarios.

Now in the web interface, once you connect to your drone, you should see four curves in the **Standard View** of the Height Readings chart as in [](#height_readings_standard_view).

<figure id="height_readings_standard_view">
    <figcaption>Standard View of the Height Readings Chart with Drone Simulated Data</figcaption>
    <img style='width:30em' src="height_readings_standard_view.png"/>
</figure>

1. Raw IR Readings: the orange curve that shows the drone simulator's simulated noisy IR readings
2. UKF Filtered Height: the blue curve that shows your UKF's height estimates, along with a shaded region indicating plus and minus one standard deviation, which is derived from the $z$ position variance in the covariance matrix
3. EMA-Smoothed Altitude: the pink curve that shows the EMA filter's estimates
4. Ground Truth Height: the black curve that is the simulated drone's actual height that we are trying to track with the UKF

If you click on the **UKF Analysis** button, the chart will change over to reveal different datasets, shown in [](#height_readings_ukf_analysis).

<figure id="height_readings_ukf_analysis">
    <figcaption>UKF Analysis View of the Height Readings Chart with Drone Simulated Data</figcaption>
    <img style='width:30em' src="height_readings_ukf_analysis.png"/>
</figure>

With this chart, we can analyze the performance of the UKF. The orange curve represents the error between the UKF and ground truth from the simulation; the closer to zero this value, the better the UKF estimates are tracking the actual altitude of the simulated drone. The blue shaded region indicates plus and minus one standard deviation of the UKF's $z$ position estimates. If the system is indeed behaving in a nice Gaussian manner and the UKF is well tuned, then we expect to see about 68% of the points in the orange dataset. Also note that on the left side of [](#height_readings_ukf_analysis), the standard deviation and error start off relatively high; this is because the filter is starting out, improving its estimates from initial values.

TODO: Make sure the state_estimator.py script that the students have is up-to-date and set up to run *their* UKF script

TODO: Make sure cross-package running of ROS nodes works (e.g., `rosrun` command in screen instead of `python`)

TODO: Consider having the student input their value for the variance of the IR sensor into the drone simulator so that they can test on their own values

**Task:** Describe how a (well-tuned) Kalman Filter outperforms an exponential moving average (EMA) filter applied to raw sensor data. Attach an image of the Height Readings graph showing the difference between your UKF and the EMA, and briefly describe the different features.

**Task:** To test out your UKF's robustness in the face of poor initialization, compare how long it takes the state estimates to converge to accurate values with good initial conditions and with poor initial conditions.
TODO: Look into the best way in which students should be characterizing filter performance. Perhaps by evaluating state estimates against MoCap ground truth and seeing if the filter's covariance matrix $\mathbf{P}$ reports accurate variances. This seems like an intuitive and mathematically rigorous enough method. Or, if students are developing their UKF on simulated data, then ground truth would be the mean values in the simulation about which we are generating noise.

## Exercise: Designing a UKF for a Ground Robot

At this point, you should have a functioning 2D UKF on your drone---congratulations! Before you get checked off for your work thus far, we want to introduce you to designing a slightly more complicated UKF for a different robot system that includes nonlinearities in the state transition and measurement functions. Here is the scenario:

Imagine a ground robot capable of moving in the $xy$-plane. Its body frame is oriented such that the origin is located at its center, the positive $x$-axis points to the right of the robot, the positive $y$-axis points forward, and the positive $z$-axis points up. The robot can control its forward acceleration $\ddot y^b$ in the body frame and its heading $\psi$ (rotation about its center). Its sensor suite consists of a forward-pointing range sensor. With this setup:

- What is a reasonable state vector?
- Define the state transition function (hint: there will be nonlinearities)
- Define the measurement function (hint: there will be nonlinearities)

**Task:** Write up your solutions to the above bullet points in $\LaTeX$ in the file `ground_robot_ukf.tex`.

TODO: Figure out a better way to frame what the state vector should be perhaps. In other words, work through this problem first and then make sure it's reasonable for students to do it.

# Check-Off {#ukf-one-spatial-dimension-checkoff status=draft}
Come to TA hours to get checked off when you have fulfilled the following:

- Relevant solutions are written up in `ukf2d_written_solutions.tex`. $\LaTeX$ source is pushed to GitHub.
- UKF is implemented in code in `student_state_estimator_ukf_2d.py` and pushed to GitHub. Your UKF can run:
    1. On simulated data. You can quantify its performance relative to simulated ground truth and see it working better than the EMA
    2. On your drone while manually moving it up and down. You can see in the web interface Height Readings chart that it performs better than the EMA when moving the drone
    3. On your drone while it is in flight, with comparable or better performance than when running the EMA filter for state estimation
- Relevant solutions are written up in `ground_robot_ukf.tex`.

Now that you have derived the mathematical design for your 2D UKF and implemented it, come to TA hours with your $\LaTeX$ document and code pushed to GitHub to get checked off. In addition to these deliverables, we will be asking you to verbally describe some of the answers you wrote up in your $\LaTeX$ document, which will enable us to better assess your understanding of the work in the project so far and your readiness for the next stage of the project. Feel free to also ask the TAs questions about the next part of the project during your check-off.