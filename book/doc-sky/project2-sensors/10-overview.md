# Project 2: Sensor Interfacing {#sensors-project status=draft}

## Overview
In this project, you will be interfacing with the sensors to extract data, parse it into useful values, and publish the data on ROS topics for use by the robot. First, you will interface with the infrared range sensor, thus providing the drone with knowledge of it's height from the ground. Then, you will interface with the camera to extract positions and velocities. In the next project, you will write a state estimator which uses all of this data you've collected to estimate the state of the drone.

## Handin
Use this link to generate a Github repo for this project. Clone the directory to your computer `git clone https://github.com/h2r/project3pid-yourGithubName.git` This will create a new folder. The _README.md_ in your repo provides short descriptions of each project file.

When you submit your assignment, your folder should contain the following files (that you modified or created) in addition to all of the other files that came with your repo:

* _answers_pid.md_
* _student_pid_class.py_
* _z_pid.yaml_
* <i>original_controller</i>
* <i>student_controller</i>
* <i>position_controller</i>

Commit and push your changes before the assignment is due. This will allow us to access the files you pushed to Github and grade them accordingly. If you commit and push after the assignment deadline, we will use your latest commit as your final submission, and you will be marked late.

```
cd project3pid-yourGitHubName
git add -A
git commit -a -m 'some commit message. maybe handin, maybe update'
git push
```

Note that assignments will be graded anonymously, so please don't put your name or any other identifying information on the files you hand in.
