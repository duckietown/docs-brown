# SLAM Background {#localization-slam-slam-background status=ready}

Congratulations! You have implemented a real-time localization algorithm for a flying drone.

While this code tends to work pretty well, consider the limitations of a localization algorithm which only works when a map of the environment is available beforehand. In the near future, we will likely see autonomous robots operating in our schools and homes. Such robots will have no access to maps of their environments beforehand; they will need to map their environments in real time!

To provide this functionality to the PiDrone, we will extend the localization particle filter such that each particle will not just estimate the path of the drone, but a map of the drone's environment.  

The algorithm that accomplishes this is called FastSLAM. A map in FastSLAM is represented by a set of **landmarks.** A Gaussian approximates the pose of the landmark. For us, this is a pose (x,y) and a 2x2 covariance matrix. In our implementation, a single landmark in the map corresponds to a single feature extracted by OpenCV.

Most SLAM algorithms seek to approximate the following probability distribution:

$p(\Theta, x^t | z^t, u^t)$  

where $\Theta$ is the map consisting of N landmarks $\Theta=\theta_1,...\theta_N$  
$x^t$ is the path of the robot $x^t= x_1,...,x_t$  
$z^t$ is the sequence of measurements $z^t= z_1,...,z_t$  
$u^t$ is the sequence of controls, $u^t= u_1,...,u_t$  

ie approximate the path of the drone and the map of its environment given all past measurements and controls.

The main mathematical insight of FastSLAM is the ability to factor the above belief distribution by landmark:  
$p(\Theta, x^t | z^t, u^t)=p(x^t | z^t, u^t)\Pi_n{p(\theta_n |x^t, z^t, u^t)}$

This factorization asserts the fact that landmark positions in the map are *conditionally independent* of one another if the path of the robot is known. Hence the product over n for each landmark $\theta_n$.

With this insight, we can represent the map with many 2-dimensional Gaussians, one for each landmark. Otherwise, as in the popular EKF SLAM algorithm, we would have to store and update a 2N-dimensional Gaussian, for N as the number of landmarks in the map. As you will see, our maps will grow to include hundreds of landmarks. Updating a covariance matrix with $(2N)^2$ entries for N=500 landmarks would not be so fun!

The basic steps of FastSLAM will closely resemble those of MC Localization: generate a set of particles and in each time step: update their positions with motion data, weight them based on their accuracy, and resample.

The following animation shows FastSLAM running on the PiDrone:    

https://www.dropbox.com/s/ywwm24ax3dxfsjo/SLAM.mp4?dl=0

In grey are all of the landmarks in the map, in blue are the features being observed by the drone during each
moment in time, and in red are the poses of the FastSLAm particles (our belief
about the location of the **drone**).

Notice that as the drone moves throughout the plane, newly observed features,
marked in blue, are added to the map as grey particles. As areas of the map
are revisited by the drone, the algorithm updates those areas with the new
information, and you can see the landmarks shift. Remember that the pose of each
landmark is filtered with an EKF, so as we revisit a landmark more times,
we incorporate more information about it and our certainty about the map increases.

Please provide answers to the following questions in answers.md

## Problem 2 - FASTSLAM questions
Q1- Why is the property of the landmark positions being conditionally independent important for FasTSLAM?

Q2- Does FASTSLAM include EKF's? If yes, how are they part of the algorithm?
