# 7D UKF Design and Implementation {#ukf-three-spatial-dimensions status=ready}
While tracking the drone's $z$ position and velocity is helpful, it is a simplified model of the drone and does not encapsulate as many of the degrees of freedom of the drone as we might like. For this reason, you are now going to develop a UKF that tracks the drone in three spatial dimensions with a 7D state vector. Your code from the 2D UKF will be a useful reference, and many parts will be reusable for the 7D UKF.

This part of the project has **two deliverables** in your `project-ukf-2020-yourGithubName` repository, which are to be accessed and submitted via GitHub Classroom:

1. A $\LaTeX$ PDF document `ukf7d_written_solutions.pdf`, generated from `ukf7d_written_solutions.tex`, with the answers to the UKF design and implementation questions.
2. Your implementation of the UKF written in the `state_estimators/student_state_estimator_ukf_7d.py` stencil code. In this stencil code file, we have placed "TODO" tags describing where you should write your solution code to the relevant problems.

## State Vector

Just as you tracked position and velocity along one axis in the 2D UKF, now you will track position and velocity along three global-frame axes. You will also track the drone's yaw value $\psi$. Changes to the drone's orientation will cause nonlinearities that the UKF was designed to address.


$$\mathbf{x}_t = \begin{bmatrix}
x \\
y \\
z \\
\dot x \\
\dot y \\
\dot z \\
\psi \end{bmatrix}$$
We don't ask you to track the drone's attitude (roll $\phi$ and pitch $\theta$), as that makes for an even larger state vector and adds complexity. Also, the IMU incorporates its own filter to produce its estimates of roll and pitch, so there may not be much benefit to adding these variables to our UKF. As such, you will use these roll and pitch values as strong estimates to inform the state transition and measurement functions.

## State Transition Function

We define a control input $\mathbf{u}_t$ populated by linear accelerations from the IMU:

$$\mathbf{u}_t = \begin{bmatrix}
\ddot x^b \\
\ddot y^b \\
\ddot z^b \\
\end{bmatrix}$$

As noted in [the background section](#ukf-background), one could treat these acceleration values as measurements instead of control inputs; for relative ease of implementation, we have chosen to use accelerations as control inputs. The linear accelerations are in the drone's body frame, denoted by the superscript $b$, so we need to rotate these vectors into the global frame based on the yaw variable that we are tracking and the IMU's roll and pitch values. This transformation will occur in the state transition function.

To accomplish this rotation, you will use quaternion-vector multiplication (to be implemented in the stencil code in the `apply_quaternion_vector_rotation()` method). What does this operation look like, and why use this instead of Euler angles? For one, Euler angles are prone to gimbal lock, which is an issue we want to avoid in robotics. Therefore, many people in robotics and other fields such as computer graphics make use of the quaternion to avoid gimbal lock and (arguably) more elegantly encode an object's orientation or rotation. Even though your drone probably will not encounter gimbal lock in its relatively constrained envelope of operation (i.e., we are not doing flips---yet!), we want to introduce you to using quaternions in a practical calculation. Here is a [visualization](https://quaternions.online/) that might help you better grasp the admittedly unintuitive idea of the quaternion.

In particular, we are interested in rotating a vector described relative to the drone's body frame into the global frame. For example, as the drone yaws, its body-frame $x$-axis will rotate relative to the global frame, so a linear acceleration value sensed by the IMU along the drone's body-frame $x$-axis will not always correspond to the same direction in the global frame. You can imagine that roll and pitch only complicate this mapping between body and global frame.

In the state transition function, you will be rotating the body-frame linear acceleration vector from the IMU into the global frame. The computation to do this with a quaternion is as follows:

$$
\mathbf{u}^g_t = \mathbf{q} \cdot \mathbf{u}_t \cdot \mathbf{q}^*
$$

where $\mathbf{u}^g_t$ is the linear acceleration control input in the global frame, $\mathbf{q}$ is the quaternion that rotates a vector from body to global frame, $\mathbf{u}_t$ is your body-frame control input that you get from the IMU, and $\mathbf{q}^*$ is the conjugate of $\mathbf{q}$. Note that, for correct dimensionality, $\mathbf{u}^g_t$ and $\mathbf{u}_t$ should be 4-element vectors to match the quaternion's $[x, y, z, w]$ components and should have the real component $w$ equal to $0$, making these vectors "pure" quaternions.

The steps to implement this rotation in the `apply_quaternion_vector_rotation()` method, then, looks something like this:

1. Create a quaternion from Euler angles using `tf.transformations.quaternion_from_euler(roll, pitch, yaw)`, with the appropriate values for roll, pitch, and yaw (radians). The output of this function is a quaternion expressed as an array of $[x, y, z, w]$. It represents the drone's orientation and can be thought of as the quaternion to rotate a vector or frame from the global frame to the body frame. We want a quaternion that does the opposite rotation.
2. Invert the quaternion to get a quaternion that rotates a vector from the body frame to the global frame. To do this, simply negate the $w$ component (i.e., the fourth element of the first quaternion).
3. Express the vector to be rotated as a "pure" quaternion, which means appending a zero to the vector.
4. Carry out $\mathbf{q} \cdot \mathbf{u}_t \cdot \mathbf{q}^*$ by applying the following functions appropriately: `tf.transformations.quaternion_multiply` and `tf.transformations.quaternion_conjugate`.
5. Drop the fourth element of the result of this computation, and return this 3-element array.

**Task (Written Section 1.2.2):** Implement the state transition function $g(\mathbf{x}, \mathbf{u}, \Delta t)$ in `ukf7d_written_solutions.tex`. Remember that for the drone, this involves kinematics, and since we are now tracking yaw and additionally considering the roll and pitch from the IMU, a rotation will be necessary so that we track state variables in the global frame. Your implementation will use quaternion-vector multiplication as described above to accomplish this rotation. We do not expect you to write out the details of the transformation, but in your notation, you should be clear about the frame in which the control input is described (e.g., you could indicate global frame by notating the control input as $\mathbf{u}^g_t$).

**Task:** Translate the state transition function into Python by filling in the `state_transition_function()` method in `state_estimators/student_state_estimator_ukf_7d.py`. Follow the "TODO"s there. Be sure to implement `apply_quaternion_vector_rotation()` as well. As usual, note the functions' type signatures for the inputs and outputs.

## Measurement Function

The measurements $\mathbf{z}_t$ that are considered are the IR slant range reading $r$, $x$ and $y$ planar position estimates and yaw estimates $\psi_{\text{camera}}$ from the camera, and the velocities along the $x$- and $y$-axes provided by optical flow, which you learned about and implemented in [the sensors project](#sensors-project). Note that in the 2D UKF, we took the IR reading to be a direct measure of altitude; here, in three spatial dimensions, you will use the roll and pitch values directly from the IMU (i.e., not estimated in our UKF) to convert between the slant range, which is what the IR sensor actually provides, and altitude in the measurement function.

$$\mathbf{z}_t = \begin{bmatrix}
r \\
x \\
y \\
\dot x \\
\dot y \\
\psi_{\text{camera}}
\end{bmatrix}$$


At the start of your 2D UKF implementation, we asked you to take into account the notion of asynchronous inputs and to do predictions and updates when these values came in. As you later found out, this approach might not yield the best results in our particular application, due to computation limitations and also poor estimates when doing dead reckoning (i.e., predicting based on the current state estimate and motion of the drone) alone in a time step. In this 7D UKF, a similar issue can arise if trying to do a prediction and update cycle in each callback. The sporadic updates, although theoretically feasible, impose the added burden of CPU load inherent in the UKF predict and update steps. A possible solution to this issue is to drop packets of data by throttling down the sensor inputs to the UKF, which will degrade our estimates across the board. Also, by implementing callbacks that block one another, there is the potential that important updates are not being executed as often as they should be, and the system can become unreliable.

The alternative solution to this issue that we have found works better and that you will implement is to reduce the amount of computation done with each sensor input. Instead of throttling the data as it comes in, you will essentially be throttling the predict-update loop---as you ended up doing in the 2D UKF---using the `-hz` flag. When new data come in, you should store these values as the most recent values for the relevant measurement variables. Then, in a single thread, a predict-update loop will be running and using these measurements. This approach suffers from the fact that the measurements will not be incorporated into the state estimate at the *exact* time at which the inputs were received, but the predict-update loop will be running at a fast rate anyway as it will only run in one thread, so the latency should be negligible. In addition, this approach should make the algorithm simpler to implement, as you will be following the standard predict-update loop model using a single measurement function and measurement noise covariance matrix. An asynchronous approach requires that specific versions of the measurement function and covariance matrix be used for each specific sensor update, as stated by Labbe in chapter 8.10.1 of [](#bib:labbe_kalman).

**Task (Written Section 1.3.2):** In `ukf7d_written_solutions.tex`, implement the measurement function $h(\mathbf{\bar x}_t)$ to transform the prior state estimate into measurement space for the given measurement vector. Be sure to convert altitude to IR slant range based on the drone's orientation in space. This requires some trigonometry with the roll and pitch angles.

**Task:** Translate the measurement function into code by filling in the `measurement_function()` method. Follow the "TODO"s there. Note the function's type signature for the inputs and outputs.

## Process Noise and Measurement Covariance Matrices

As in the 2D UKF, we do not expect you to derive reasonable values for the process noise.

**Task (Written Section 1.3.3):** In `ukf7d_written_solutions.tex`, define the measurement noise covariance matrix with reasonable estimates for the variances of each sensor input. You already have an estimate for the IR sensor variance that you experimentally determined in the previous part of the project; for the other sensor readings, you can provide intuitive estimates and potentially attempt to later derive experimental values for these variances if your filter is not performing well.

**Task:** Enter these sample variance values into the code for `self.ukf.R` in the `initialize_ukf_matrices()` method.

## Initialize the Filter
As with the 2D UKF, we must initialize our filter before it can engage in its predicting and updating.

**Task:** Initialize the state estimate $\mathbf{x}_t$ and the state covariance matrix $\mathbf{P}_t$ with values as sensor data come in.

## Asynchronous Inputs
We touched upon this in the **Measurement Function** section. To handle asynchronous inputs, you should update instance variables with the most recent data collected and run a loop to form predictions and updates with these data.

**Task:** Implement the predict-update loop. It might be useful to refer to the `rospy` documentation on setting loop rates and sleeping.

**Task:** Complete any remaining "TODO"s in the 7D UKF source code.

## Tune and Test the Filter
It is now time to put your 7D UKF to the test.

### In Simulation

To run your 7D UKF with simulated data, you need to run ROS on your Raspberry Pi and terminate certain nodes upon running the screen:

- `flight_controller_node.py`
- `vision_flow_and_phase.py`

The simulation is only in two dimensions in the $xy$-plane, so to also test $z$ position estimates, you should keep the `infrared_pub.py` node running to see your filter work on real IR data.

Next, in the state estimator screen, terminate the current process and then run the following command:

    $ python state_estimator.py --student -p ukf7d -o simulator ema --sdim 2

If performance is clearly sub-optimal, consider using the `-hz` flag.

This command will run your 7D UKF as the primary state estimator, along with the 2D drone simulator and the EMA filter for comparison. If you do not want to run the EMA filter, simply omit the `ema` argument when running the `state_estimator.py` script.

**Task:** Make sure your UKF is producing reasonable outputs, especially in the **Top View** chart in which the simulation and its nonlinear behavior are occurring. You should qualitatively feel confident that your UKF marker (the blue marker) is more closely tracking the Ground Truth marker (black) with less noise than the Raw Pose Measurement marker (orange).

### Manually Moving the Drone

In this part of the project, you will move your drone around with your hand, holding it above a highly-textured planar surface so that the downward-facing camera can use its optical flow and position estimation to provide information about the drone's pose and twist in space. You should ensure that the following nodes are running:

- `flight_controller_node.py`
- `infrared_pub.py`
- `vision_flow_and_phase.py`

Then, you should run your UKF with this command:

    $ python state_estimator.py --student -p ukf7d -o ema

using the `-hz` flag as needed.

**Task:** Use the web interface to verify visually that the height estimates and the $x$, $y$, and yaw estimates appear to have less noise than the sensor readings, and that these estimates appear to track your drone's actual pose in space. Compare your UKF to the EMA estimates for altitude and the raw camera pose data in the **Top View** chart.




### In Flight

Now you are going to fly with your 7D UKF, using both velocity control and position hold.

**Task:** Test your drone's stability in position hold and velocity control 1) while running just the EMA filter for state estimation and 2) while running your 7D UKF. You can use the web interface to move your drone around and send it other commands.

## Final Hand-In
Before the project deadline, you must ensure that final versions of your solution files and code are handed in via GitHub Classroom. These files are:

**From the 2D UKF section:**

- `ukf2d_written_solutions.pdf` (compiled from `ukf2d_written_solutions.tex`)
- `student_state_estimator_ukf_2d.py` in the `state_estimators` directory

**From the 7D UKF section:**

- `ukf7d_written_solutions.pdf` (compiled from `ukf7d_written_solutions.tex`)
- `student_state_estimator_ukf_7d.py` in the `state_estimators` directory

Then come to TA hours to show us your working UKF code. Note that we will ask you to explain one random TODO section that you filled out.
