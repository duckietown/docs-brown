# Our Implementation: UKF in Three Spatial Dimensions {#ukf-three-spatial-dimensions status=draft}

While tracking the drone's $z$ position and velocity is helpful, it is a simplified model of the drone and does not encapsulate as many of the degrees of freedom of the drone as we might like. For this reason, you are now going to develop a UKF that tracks the drone in three spatial dimensions with a 7D state vector. A lot of your code from your 2D UKF will remain the same in the 7D UKF.

This part of the project has **two deliverables** in the `pidrone_project2_ukf` repository, which are to be accessed and submitted via GitHub Classroom:

1. A $\LaTeX$ document `ukf7d_written_solutions.tex` with the answers to the UKF design and implementation questions.
2. Your implementation of the UKF written in the `StateEstimators/student_state_estimator_ukf_7d.py` stencil code. In this stencil code file, we have placed "TODO" tags describing where you should write your solution code to the relevant problems.

## State Vector

Just as you tracked position and velocity along one axis in the 2D UKF, now you will track position and velocity along three global-frame axes. You will also track the drone's yaw value $\psi$. Changes to the drone's orientation will cause nonlinearities that the UKF was designed to address.

$$
\mathbf{x} = \begin{bmatrix}
x \\
y \\
z \\
\dot x \\
\dot y \\
\dot z \\
\psi \end{bmatrix}
$$

We don't ask you to consider the drone's attitude (roll $\phi$ and pitch $\theta$), as that makes for an even larger state vector and adds complexity. You can assume that roll and pitch are zero. Also, the IMU incorporates its own filter to produce its estimates of roll and pitch. If you wish, you may attempt to use these roll and pitch values as strong estimates to inform the state transition and measurement functions; that said, this leads to more advanced computations that we would recommend attempting after developing the UKF with the assumption of zero degrees roll and pitch.

## State Transition Function
**Task (Written Section 1.2.2):** Implement the state transition function $g(\mathbf{x}, \mathbf{u}, \Delta t)$ in `ukf7d_written_solutions.tex`. Remember that for the drone, this involves kinematics, and since we are now tracking yaw, there will also be some trigonometry.

**Task:** Translate the state transition function into Python by filling in the `state_transition_function()` method in `StateEstimators/student_state_estimator_ukf_7d.py`. Follow the "TODO"s there. Note the function's type signature for the inputs and outputs.

## Measurement Function
The measurements $\mathbf{z}_t$ that are considered are the IR slant range reading $r$, $x$ and $y$ planar position estimates and yaw estimates $\psi_{\text{camera}}$ from the camera, and the velocities along the $x$- and $y$-axes provided by optical flow.

At the start of your 2D UKF implementation, we asked you to take into account the notion of asynchronous inputs and to do predictions and updates when these values came in. As you later found out, this approach might not yield the best results in our particular application, due to computation limitations and also poor estimates when doing dead reckoning (i.e., predicting based on the current state estimate and motion of the drone) alone in a time step. In this 7D UKF, a similar issue can arise if trying to do a prediction and update cycle in each callback. The sporadic updates, although theoretically feasible, impose the added burden of CPU load inherent in the UKF predict and update steps. A possible solution to this issue is to drop packets of data by throttling down the sensor inputs to the UKF, which will degrade our estimates across the board. Also, by implementing callbacks that block one another, there is the potential that important updates are not being executed as often as they should be, and the system can become unreliable.

The alternative solution to this issue that we have found works better and that you will implement is to reduce the amount of computation done with each sensor input. Instead of throttling the data as it comes in, you will essentially be throttling the predict-update loop. When new data comes in, you should store these values as the most recent values for the relevant measurement variables. Then, in a single thread, a predict-update loop will be running and using these measurements. This approach suffers from the fact that the measurements will not be incorporated into the state estimate at the *exact* time at which the inputs were received, but the predict-update loop will be running at a fast rate anyway as it will only run in one thread, so the latency should be negligible. In addition, this approach should make the algorithm simpler to implement, as you will be following the standard predict-update loop model using a single measurement function and measurement noise covariance matrix. An asynchronous approach requires that specific versions of the measurement function and covariance matrix be used for each specific sensor update, as stated by Labbe in chapter 8.10.1 of [](#bib:labbe_kalman).

TODO: Make sure that the 7D UKF works well when actually removing the IMU yaw variable from the measurement vector/function. It has worked well when not used to update `last_measurement_vector[5]`, but it is worth making sure that amending the structure will not have adverse effects.

TODO: Implement the predict-update loop in the main thread with a while loop that uses rospy Rate and sleep, probably. Test this out and then write about it in this project description for the students to implement.

**Task (Written Section 1.3.2):** In `ukf7d_written_solutions.tex`, implement the measurement function $h(\mathbf{\bar x})$ to transform the prior state estimate into measurement space for the given measurement vector.

**Task:** Translate the measurement function into code by filling in the `measurement_function()` method. Follow the "TODO"s there. Note the function's type signature for the inputs and outputs.

## Process Noise and Measurement Covariance Matrices

As in the 2D UKF, we do not expect you to derive reasonable values for the process noise.

**Task (Written Section 1.3.3):** In `ukf7d_written_solutions.tex`, define the measurement noise covariance matrix with reasonable estimates for the variances of each sensor input. You already have an estimate for the IR sensor variance that you experimentally determined in the previous part of the project; for the other sensor readings, you can provide intuitive estimates and potentially attempt to later derive experimental values for these variances if your filter is not performing well.

**Task:** Enter these sample variance values into the code for `self.ukf.R` in the `initialize_ukf_matrices()` method.

## Initialize the Filter
As with the 2D UKF, we must initialize our filter before it can engage in its predicting and updating.

**Task:** Initialize the state estimate $\mathbf{x}$ and the state covariance matrix $\mathbf{P}$ with values as sensor data come in.

## Asynchronous Inputs
We touched upon this in the **Measurement Function** section. To handle asynchronous inputs, you should update instance variables with the most recent data collected and run a loop to form predictions and updates with these data.

**Task:** Implement the predict-update loop. It might be useful to refer to the `rospy` documentation on setting loop rates and sleeping.

TODO: Make sure such `rospy` rate-usage and sleeping works well

## Tune and Test the Filter
It is now time to put your 7D UKF to the test.

### In Simulation
TODO: Make sure the 2D simulation works well and that the 7D UKF works well with it.

To run your 7D UKF with simulated data, you need to run ROS on your Raspberry Pi and terminate certain nodes upon running the `pi.screenrc` file:

- `flight_controller_node.py`
- `vision_flow_and_phase.py`

The simulation is only in two dimensions in the $xy$-plane, so to also test $z$ position estimates, you should keep the `infrared_pub.py` node running to see your filter work on real IR data.

Next, in a free screen, navigate to the `pidrone_project2_ukf` repository and run the following command:

    duckiebot $ python state_estimator.py -p ukf7d -o simulator ema --sdim 2

If performance is clearly sub-optimal, consider using the following flags:
- `--ir_throttled`
- `--imu_throttled`
- `--optical_flow_throttled`
- `--camera_pose_throttled`

This command will run your 7D UKF as the primary state estimator, along with the 2D drone simulator and the EMA filter for comparison. If you do not want to run the EMA filter, simply omit the `ema` argument when running the `state_estimator.py` script.

TODO: Consider instead having them run it from `pidrone_pkg` (or from their package) but use the `pidrone_pkg`'s `state_estimator.py` script. Have the `state_estimator.py` script take in a `--student` flag, for example?

TODO: Test out 7D UKF unthrottled

**Task:** Make sure your UKF is producing reasonable outputs, especially in the **Top View** chart in which the simulation and its nonlinear behavior are occurring. You should qualitatively feel confident that your UKF marker (the blue marker) is more closely tracking the Ground Truth marker (black) with less noise than the Raw Pose Measurement marker (orange). We will ask you to show us your UKF's simulation performance at the final check-off.

TODO (NOTE): Don't require students to test out with localization, as they will not have implemented localization at this point so we don't necessarily want to get ahead of ourselves. Also, running both localization and the 7D UKF onboard at the same time might be too much CPU load to be able to get accurate estimates, either from localization or perhaps from the UKF. This was roughly tested out, and the UKF estimates were tracking the localization pose estimates pretty smoothly, but the drone was not too stable, perhaps because running both localization (which is already a CPU-intensive process to run onboard) and the UKF onboard at the same time might be a lot for the CPU to handle. More integration testing should be done if we want to successfully run both a three-spatial-dimension UKF and localization onboard.

# Check-Off {#ukf-three-spatial-dimensions-checkoff status=draft}
Come to TA hours to get checked off when you have fulfilled the following:

- Relevant solutions are written up in `ukf7d_written_solutions.tex`. $\LaTeX$ source is pushed to GitHub.
- UKF is implemented in code in `student_state_estimator_ukf_7d.py` and pushed to GitHub. Your UKF can run:
    1. On simulated data. We will ask you to show us your drone's UKF in simulation and make sure that it tracks ground truth reasonably well [_TODO: Can we make this more quantitative? UKF-ground-truth errors for x and y position, perhaps?_]
    2. On your drone while manually moving it up and down, forward and back, left and right, and giving it a yaw angle. You can see in the web interface Standard View of the Height Readings chart that it performs better than the EMA when moving the drone up and down and that it performs better than raw camera pose data (from `vision_flow_and_phase.py`) in the **Top View** chart
    3. On your drone while it is in flight in velocity control mode and position hold mode

TODO: Consider, instead of doing a check-off for the end of the project, having students submit videos of the functionality of their drones running the 7D UKF.

Again, make sure your code and `.tex` file are submitted in GitHub Classroom.
