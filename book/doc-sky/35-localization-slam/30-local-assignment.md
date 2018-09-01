# Localization Assignment {#localization-slam-assignment status=ready}

First, you will complete a series of quick exercises which will guide you through implementing a simplified particle filter. You will be given two files:

    particle_filter.py
    animate_particle_filter.py

In particle_filter you will implement a particle filter which causes a set of randomly generated points on a 2d plane to converge on a specific point. particle_filter will write the particles' poses to a text file, which animate_particle_filter will read and use to generate an animation.

**Problem 1: Setup**
Define a Particle class to represent the particles in the filter. Each particle should store its position (x,y) and its weight.

Define a ParticleFilter class to store the set of particles, the desired pose, and the methods which operate on the particle set.  Create an __init__ method which takes the number of particles as input and creates a set of particles at random positions.

**Problem 2: Motion**
Implement a method for the ParticleFilter class which adds some random Gaussian noise to the x and y positions of each particle in the filter. *Hint:* try numpy.random.normal.

**Problem 3: Measurement Update**
Implement a method for the ParticleFilter class which sets the weight of each particle inversely proportional to the particle's distance from the desired pose.

**Problem 4: Resampling**
Implement a method for the ParticleFilter class which uses the particle weights to resample the particles and create a new set of particles. The goal is for each particle to be resampled with a probability that corresponds to its weight, therefore the particle set converges on the desired pose (particles closest to the target pose are more likely to be resampled). You will want to use copy.deepcopy to create a copy of each re-sampled particle.

*Hint:* try using numpy.random.multinomial to generate the set of new samples.

**Problem 5: Finishing Up**
Finally, implement a method for the ParticleFilter class which updates the filter by calling the motion method, the measurement method, and the resampling method. You may test your implementation by running particle_filter.py, then running animate_particle_filter.py to view the animation. You should see the set of particles converge to the desired point.

**Problem 6: Optimization**
Now that your filter is running, let's consider how we can optimize this process so that localization will run quickly in real time on the PiDrone.

Python data structures and their operations are relatively slow compared to their Numpy counterparts because Numpy is written in C which is a fast language. You will use Numpy arrays to avoid storing the set of particle poses and their weights as lists of Python objects. Remove your Particle class, and replace it with two Numpy arrays which hold the sets of all particle poses and weights. Adjust the rest of your code accordingly.

Once the particle filter animation is working properly, you are ready to implement localization on the drone!


**Implement Localization on the PiDrone**
You will be given two files:

    vision_localization_onboard.py
    localization_helper.py

vision_localization_onboard runs localization on the drone and is complete, you will not need to implement any code in that file. However, you may adjust the NUM_PARTICLE and NUM_FEATURE values to experiment with the speed/accuracy tradeoff concerning the number of particles in the filter and the number of features extracted by OpenCV. You may also edit this file if you need to change the map over which you want to localize.

localization_helper contains the particle filter class and its methods. Many of the methods are not implemented. The docstrings describe the intended functionality of each function, and the TODOs indicate tasks to be completed. Your assignment is to follow the TODOs and implement the missing functionality of the particle filter. Much of the code you just wrote can be used here!

## Testing
To test the functionality of your localization code, you may fly the drone while running vision_localization_onboard in the vision window with your localization_helper code in the scripts folder. Follow the Pidrone Vision Instructions to see how to change the map. You should see poses printed out which correspond to the drone's position over the map.

You may also use animate_particle_filter.py to view the animation of your particle filter. Print the (x,y) pose of each particle on separate lines in a text file to be read by animate_particle_filter, put x and y pose coordinates on separate lines. Make sure you adjust animate_particle_filter.py to reflect the number of particles you are using!

## Checkoff
Before you come to TA hours to have your project graded, you should verify that your code has the following functionality:

 1. You can run vision_localization_onboard.py and take off with your drone.
 2. While flying, you can hit 'r' and the poses will begin printing to the terminal. You can hit 'r' again and localization will restart.
 3. While flying, you can hit 'p' to toggle position hold on and off.
 4. Run picam_localization.py while holding the drone over a mapped area. Do not arm the drone. As you move the drone around, verify that the poses reflect the movement. Verify visually that the poses are close to the actual position of the drone in the map. For example, if you are holding the drone above the bottom left corner of the mapped area, the pose should be close to (0,0).
