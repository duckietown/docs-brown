# Localization Assignment {#localization-slam-localization-assignment status=ready}

## Getting Set Up
You should have cloned the GitHub Classroom link to receive the deliverables
for this project.

You should receive a directory named "project-localization-yourGithubName." The only part
of this assignment which you must run on the drone is localization (the last
assignment on this page). To do this, place your directory in the /ws/src folder
on your drone. You should find the "package.xml" and "CMakeLists.txt" files which you will need to modify
to build the package. On line 3 of "package.xml" you should replace
yourGithubName with your GitHub name so it matches the name of your directory. Do the same
on line 2 of "CMakeLists.txt" Finally, you should navigate to the /ws folder and run

    catkin_make --pkg project-localization-yourGithubName

to build your package so it is ros-runnable from the pidrone_pkg. You should only need to
do this step one time.


## Dependencies
In order to complete this project, we will make use of the following libraries: Numpy
for computations,  OpenCV for computer vision, and MatPlotLib for creating
plots and animations. You are welcome to run on your drone the parts which do
not require visualization, ie the OpenCV assignment.
However, the particle filter assignment will require you to view a MatPlotLib
animation. To accommodate this, you may either install the required dependencies
on your own computer (optional!) or work on a department machine which already
has them. If you install OpenCV yourself, make sure the version is 2.4.9.
The easiest way to work on this project is to work over ssh on your
computer and use XQuartz (what the -Y is for when you type ssh -Y) which will
allow you to view animations over ssh. As a reminder, to access
your account on the department machines, open a terminal and run "ssh -Y your_login@ssh.cs.brown.edu."
You may use cyberduck or your preferred method to transfer files from your computer to the
department machines.

## Particle Filter
First, you will complete a series of quick exercises which will guide you through implementing a simplified particle filter. This part of the assignment must be completed on a computer with matplotlib and numpy installed. You will be given two files:

    student_particle_filter.py
    animate_particle_filter.py

In student_particle_filter you will implement a particle filter which causes a set of randomly generated points on a 2d plane to converge on a specific point. student_particle_filter will write the particles' poses to a text file, which animate_particle_filter will read and use to generate an animation.

Note that there are more detailed instructions for each step in the comments of
student_particle_filter.

**Problem 1: Setup**
Define a Particle class to represent the particles in the filter. Each particle should store its position (x,y) and its weight.

Define a ParticleFilter class to store the set of particles, the desired pose, and the methods which operate on the particle set.  Create an __init__ method which takes the number of particles as input and creates a set of particles at random positions.

**Problem 2: Motion**
Implement a method for the ParticleFilter class which adds some random Gaussian noise to the x and y positions of each particle in the filter. Be sure that the noise is different for each particle. *Hint:* try numpy.random.normal.

**Problem 3: Measurement Update**
Implement a method for the ParticleFilter class which sets the weight of each particle inversely proportional to the particle's distance from the desired pose.

**Problem 4: Test**
Try running your code! If it works properly, the particle poses should be written to a file
called "particle_filter_data.txt." You can then run the file "animate_particle_filter" to view
an animation of your particle filter converging on the desired pose which you set.

**Problem 5: Optimization OPTIONAL STEP**
Now that your filter is running, let's consider how we can optimize this process so that the localization particle filter will run quickly in real time on your drones.

Python data structures and their operations are relatively slow compared to their Numpy counterparts because Numpy is written in C. You will use Numpy arrays to avoid storing the set of particle poses and their weights as lists of Python objects. You may comment out the Particle class entirely and replace the list of particle objects with two Numpy arrays for poses and weights stored in the ParticleSet class. Adjust the rest of the code accordingly. This step is meant to help you understand the optimizations (which are done in the same way) in the localization code.

## OpenCV
This part of the assignment may be completed on your drones, or any computer with OpenCv and NumPy.

Now we that know the basics of how a particle filter uses weights and resampling to converge on a target, we need to address how to use OpenCV to estimate the motion and global position of the flying drone. To do this, you will complete a short assignment using OpenCV functions to compute the translation in the plane between two drone poses, represented by two overlapping images taken on a real drone. You will be provided with the following files:

    image_A.jpg
    image_B.jpg
    student_compute_displacement.py

student_compute_displacement.py will indicate the infrared reading taken by the drone at the time images A and B were taken. This is important because the real-world dimensions of a pixel in the image will vary based on the height of the drone. Why is this?

Your job is to write code in student_compute_displacement.py that will extract features from both images and compute a transformation between them. Use this transformation to compute the x,y, and yaw displacement in *meters* between the two images. This is exactly how you will implement the motion model for localization: we consider the meter displacement between two drone images to be the motion of the drone between the poses at which the images were taken.

## Implement Localization on the PiDrone
We are now ready to implement localization on the drone.

You will be given two files:

    student_run_localization.py
    student_localization_helper.py

student_run_localization runs localization on the drone and is complete, you will not need to implement any code in that file. However, you may adjust the NUM_PARTICLE and NUM_FEATURE values to experiment with the speed/accuracy tradeoff concerning the number of particles in the filter and the number of features extracted by OpenCV. You may also edit this file if you need to change the map over which you want to localize.

student_localization_helper contains the particle filter class and its methods. Many of the methods are not implemented. The docstrings describe the intended functionality of each function, and the TODOs indicate tasks to be completed. Your assignment is to follow the TODOs and implement the missing functionality of the particle filter. Much of the code you just wrote can be used here!

Tip: we recommend that you read through the parts of the code which we are not asking you to implement,
as this will help you to understand what is going on with the code and will likely save you
debugging time. For example, we are not asking you to implement "resample_particles" or "initialize_particles"
for localization, but it might help you to understand how they work! The same goes for the SLAM project.

Note that for both this part of the assignment and for SLAM, there is not any "correct" universal
implementation of the code as long as your solutions work.

## Testing
To test the functionality of your localization code, you may fly the drone while running

    rosrun project_localization_yourGithubName student_run_localization.py

in the vision window. Follow the Mapping and Localization instructions in the operations manual to see how to change the map. You should see poses printed out which correspond to the drone's position over the map.

You may also use animate_particle_filter.py to view the animation of your particle filter. Print the (x,y) pose of each particle on separate lines in a text file to be read by animate_particle_filter, put x and y pose coordinates on separate lines. Make sure you adjust animate_particle_filter.py to reflect the number of particles you are using! (using the visualizer here is optional)

## Checkoff
We will verify that your code has the following functionality:

 1. You can run student_run_localization.py and take off with your drone.
 2. While flying, you can hit 'r' and the poses will begin printing to the terminal. You can hit 'r' again and localization will restart.
 3. While flying, you can hit 'p' to toggle position hold on and off.
 4. Run student_run_localization.py while holding the drone over a mapped area. Do not arm the drone. As you move the drone around, verify that the poses reflect the movement. Verify visually that the poses are close to the actual position of the drone in the map. For example, if you are holding the drone above the bottom left corner of the mapped area, the pose should be close to (0,0).
