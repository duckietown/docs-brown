# Project {#ukf-project status=notready}

## UKF in One Spatial Dimension
You will start by implementing this state estimation algorithm with a simple one-dimensional model of the drone's motion to estimate its position and velocity along the vertical axis. Later on in the project, we will expand the model to three spatial dimensions.

### Outline of Background Information to Cover
Before we dive into the UKF, there are some foundations that we should build up:

1. The Bayes Filter

2. Gaussians

3. The Kalman Filter

### The Bayes Filter

**TODO: Fill in with Luke's description**

### Gaussians

**TODO: Brief description of Gaussians as needed to discuss the Kalman Filter. Perhaps show multivariate Gaussians as well, e.g., with a 3D plot and an error ellipse to illustrate 2D Gaussian**

### The Kalman Filter
#### High-Level Description of the Kalman Filter Algorithm

The Kalman Filter's main goal is to fuse measurement readings with predicted states. For example, if we know (with some degree of uncertainty) that the drone is moving upward, then this knowledge can inform us about the drone's position at the next time step. We can form a **prediction** of the drone's state at the next time step given the drone's current state and any control inputs that we give the drone. Then, at the next time step when we receive a new measurement, we can perform an **update** of our state estimate. In this update step, we look at the difference between the new measurement and the predicted state. This difference is known as the *residual*. Our new state estimate (referred to in literature as the *posterior*) lies somewhere between the predicted state, or *prior*, and the measurement; the scaling factor that accomplishes this is known as the *Kalman gain*. Figure 1 depicts this process of prediction and measurement update.

![residual_chart](residual_chart.png)

***Figure 1. Interaction of the Prediction and Measurement in a Kalman Filter[](#bib:labbe_kalman)***

##### State Vector and Covariance Matrix

The KF accomplishes its state estimate by tracking certain **state variables** in a state vector, such as position and velocity along an axis, and the **covariance matrix** corresponding to the state vector. In the first part of this project, we define the state vector as:

$$\mathbf{x}=\begin{bmatrix}
z \\
\dot z
\end{bmatrix}$$

where $z$ and $\dot z$ are the position and velocity of the drone along the z-axis, respectively. The state vector tracks the mean $\mu$ of each state variable, which we assume is normally distributed about $\mu$. For this state vector, then, we define the covariance matrix as:

$$\mathbf{P}=\begin{bmatrix}
\sigma^2_z & \sigma_{z,\dot z} \\
\sigma_{\dot z,z} & \sigma^2_{\dot z}
\end{bmatrix}$$

where $\sigma^2_z = \text{Var}\left( z \right)$, for example, denotes the variance in the position estimates and $\sigma_{\dot z,z} = \sigma_{z,\dot z} = \text{Cov}\left( z, \dot z \right)$ denotes the covariance between the position and velocity estimates. Typically, position and velocity are positively correlated, as a positive velocity indicates that the drone will likely be at a more positive position at the next time step.

**TODO: Write out the recursive algorithm**

#### A Closer Look at Some of the Math of the Kalman Filter

**TODO: Figure out what to put here**

### Designing a Kalman Filter

To apply a Kalman Filter to a specific robot, there are certain parts of the algorithm that we need to define. The first aspect of the KF design process specific to the robot application is the selection of state variables to track in the state vector. Additionally, the motion model of the robot and its sensor suite demand careful thought when designing a KF: the former determines the state transition function, and the latter specifies the measurement function. We will be going over these design decisions in this section.

#### State Vector Design






