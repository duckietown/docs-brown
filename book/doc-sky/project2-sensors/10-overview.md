# Project 2: Extracting Data from Sensors {#part:sensors status=ready}

## Overview
In this project, you will be interfacing with the drone's sensors to collect the data necessary for the drone to perform all of its capabilities. In part one, you will write a ROS node to extract height readings from the IR sensor. In part two, you will use optical flow and OpenCV tools to calculate velocity and position estimates from images taken by the downward-facing Raspberry Pi camera.

## Handin
Use this link to generate a Github repo for this project. On your drone, navigate to `~/ws/src`. Clone the directory to your drone `git clone https://github.com/h2r/project2sensors-yourGithubName.git` This will create a new folder. Then change directories back to `~/ws/` and run `catkin_make --pkg project2sensors-yourGitHubName`.

When you submit your assignment, your folder should contain the following files (that you modified or created) in addition to all of the other files that came with your repo:

* _student_infrared_pub.py_
* _student_analyze_flow.py_
* _student_analyze_phase.py_

Commit and push your changes before the assignment is due. This will allow us to access the files you pushed to Github and grade them accordingly. If you commit and push after the assignment deadline, we will use your latest commit as your final submission, and you will be marked late.

```
cd project2sensors-yourGitHubName
git add -A
git commit -a -m 'some commit message. maybe handin, maybe update'
git push
```

Note that assignments will be graded anonymously, so please don't put your name or any other identifying information on the files you hand in.
