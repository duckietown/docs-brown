# Part 5: Set up Docker {#docker status=ready}

To make it possible for you to develop your own code on the drone, you
need to set up a docker workspace with our source code, pidrone_pkg.

To do this, you need to clone this repository to your drone's SD card,
then build the Docker image needed to run the software (which is back
on an older version of ROS, ROS Kinetic).

Run `ssh duckie@yourdrone` to ssh into your drone.  The password is
`quackquack`.  Then from your home directory run the following commands:


```
sudo apt install rake
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/h2r/pidrone_pkg
cd pidrone_pkg
git checkout ente
```

The next step will take a long time because it has to download all the
preqrequisites for the image.  Make sure your Pi is plugged into
external power either through the battery or through the USB power
supply.

```
rake build
# this step is fast
rake create
```

Once these steps are complete, you can start the container and go inside by running 
```
rake start
```

Once in the container, run
```
screen -c pi.screenrc
```

This will start a screen session with each of the ros nodes needed to
run the drone and make it fly. 

