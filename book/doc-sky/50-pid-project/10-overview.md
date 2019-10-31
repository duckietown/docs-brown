# Project 3: Implementing an Altitude PID Controller {#pid-overview status=ready}

## Overview
For this project, you will be implementing a one-dimensional PID controller to control the drone's altitude (movement along the drone's z-axis). In part one, you will write your PID class and test it using a drone simulator. In part two, you will answer questions about control. 

There are three sections that are not required for this project. In Appendix A, we describe how to steady the drone's planar motion using the planar PIDs. In Appendix B, we explain how to transfer the altitude PID controller you wrote in part 1 to your drone and tune it to achieve stable flight. In Appendix C, we show how to use your tuned PIDs to control the position of the drone.

## Handin
Use this [link](https://classroom.github.com/a/DU3P2-w1) to generate a Github repo for this project. Clone the directory to your computer `git clone https://github.com/h2r/project-pid-implementation-yourGithubName.git` This will create a new folder. The _README.md_ in your repo provides short descriptions of each project file.

When you submit your assignment, your folder should contain the following files (that you modified) in addition to all of the other files that came with your repo:

* _answers_pid.md_
* _student_pid_class.py_
* _z_pid.yaml_

Commit and push your changes before the assignment is due. This will allow us to access the files you pushed to Github and grade them accordingly. If you commit and push after the assignment deadline, we will use your latest commit as your final submission, and you will be marked late.

```
cd project-pid-implementation-yourGitHubName
git add -A
git commit -a -m 'some commit message. maybe handin, maybe update'
git push
```

Note that assignments will be graded anonymously, so please don't put your name or any other identifying information on the files you hand in.
