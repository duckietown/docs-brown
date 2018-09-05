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
The measurements $\mathbf{z}_t$ that are considered are the IR slant range reading $r$, $x$ and $y$ planar position estimates and yaw estimates $\psi_{\text{camera}}$ from the camera, the velocities along the $x$- and $y$-axes provided by optical flow, and the IMU's drift-prone yaw reading $\psi_{\text{IMU}}$. The IMU integrates angular velocities to arrive at an estimate of yaw, so it will slowly drift over time. One possible way to try countering this effect is to add the IMU yaw bias to the state vector and track it as it drifts, using the camera's yaw estimate to estimate this IMU bias term. You don't have to implement this bias tracking.

The measurement vector $\mathbf{z}_t$ does not always contain all of the measurement variables in the real system at any given time that a measurement is read, and this is due to the fact that sensors generally operate at different rates and output data asynchronously. Since $\mathbf{z}_t$ changes size, the measurement function $h(\mathbf{x})$ and the measurement covariance matrix $\mathbf{R}$ must both be updated to reflect the current measurement input, as stated by Labbe in chapter 8.10.1 of [](#bib:labbe_kalman).

Since we have four sources of sensor inputs with different data rates, we perform measurement updates asynchronously in each of four callbacks.

TODO: Modify this section to be consistent with recent findings that the 7D UKF actually seems to operate better without doing a prediction and measurement update in each callback.

**Task (Written Section 1.3.2):** In `ukf7d_written_solutions.tex`, implement each measurement function $h(\mathbf{\bar x})$ to transform the prior state estimate into measurement space for each set of measurements.

**Task:** Translate the measurement functions into code by writing four methods that follow the type signature of the measurement function you implemented in your 2D UKF.

## Process Noise and Measurement Covariance Matrices
In addition to having four measurement functions, you must define four measurement covariance matrices $\mathbf{R}$ to accompany these measurement updates.

**Task (Written Section 1.3.3):** In `ukf7d_written_solutions.tex`, define each covariance matrix with reasonable estimates for the variances of each sensor input. You already have an estimate for the IR sensor variance that you experimentally determined in the previous part of the project; for the other sensor readings, you can provide intuitive estimates and potentially attempt to later derive experimental values for these variances if your filter is not performing well.

**Task:** Enter these sample variance values into

## Initialize the Filter
TODO

## Asynchronous Inputs
TODO

## Tune and Test the Filter
TODO: Write this section and include description on quantitatively evaluating the performance of the filter