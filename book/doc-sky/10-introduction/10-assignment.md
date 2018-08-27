# Assignment {#introduction-assignment status=ready}

This assignment gives an introduction to our course and
reviews some basic material you will need. *Hand ins will be
noted in italics.*

## Course Form

Fill out the
[form](https://docs.google.com/forms/d/1H9RmjkpoRjVK3JAbkEogyKDuwZQWIR6a7M4KQng4Tf0/viewform?edit_requested=true)
in order to apply for admission to the course. 

## Collaboration Policy

Please read and sign the [collaboration policy for
CS1951R](collab_policy.pdf).*Submit the signed pdf with filename
`collaboration_policy.pdf`.*

## Safety Policy

Please read and sign the [safety policy for CS1951R](safety_policy.pdf).
*Submit the signed pdf with filename `safety_policy.pdf`*

## Motivations (20 points)

*Submit the answers to these questions in `answers.txt`*

Before you start putting a lot of time into this course, it is important to
figure out what you will get out of the course.  Think about what you expect to
learn from this course and why it is worth investing a lot of time.  What do
you hope to learn that you can take away for the next ten or twenty years of
your career?

1. What is a robot? What is a machine? What is a vehicle?
2. Is a car a robot? How is my drone a robot?
3. If I can fly a drone by remote, what can I get out of programming it?

## Matrices and Transformations (20 points)

*Write the answers to these questions in the corresponding section of `answers.txt`.*

For this problem we strongly recommend you do these calculations by hand,
because they are warmup questions designed to remind you of some of the
prerequisite material for the class.

1. Multiply the matrix by the following vector: 
  $$\begin{bmatrix}1 & 0 & 1\\
                 0 & 1 & 2\\
                 0 & 0 & 1\end{bmatrix} 
  \times 
  \begin{bmatrix}0\\
                 0\\
                 1\end{bmatrix}$$

2. Multiply the matrix by the following vector:
   $$\begin{bmatrix}0\\
                  0\\
                  1\end{bmatrix} 
   \times 
   \begin{bmatrix}1 & 0 & 4\\
                  0 & 1 & 10\\
                  0 & 0 & 1\end{bmatrix} $$

3. Imagine a robot is at $$(0x,0y)$$. It uses a sensor to detect an object on
   a distance of $$6\mbox{m}$$ and a heading of $$30^{\circ}$$. The facing
   toward positive numbers on the Y axis would be $$0^{\circ}$$, and the
   degrees increase when turning clockwise. Where is the obstacle relative to
   the robot? Give coordinates and draw your answer on a map.

## FAA Rules (20 points)

Read the [FAA website](https://www.faa.gov/uas/) on Unmanned Aircraft Systems.
Provide short answers to the following questions.

*Write the answers to these questions in the corresponding sections in `answers.txt`*

1. What class of airspace is the area around the CIT? (You might find it
   easiest to use the [B4UFLY Smartphone
   App](https://www.faa.gov/uas/where_to_fly/b4ufly/).

2. What is the closest airport to the CIT? Hint: Make sure to check for
   heliports as well.

3. Is it okay to fly your drone outside the CIT? Why or why not? 

4. What are some risks of drone flight? How could people get hurt with the
   robot?

## Case Study (20 points)

*Write your answers in the corresponding section in `answers.txt`*

Pick one of these robots that was used to solve a real-world problem and answer
the questions below.
- [Baxter](https://www.rethinkrobotics.com/baxter/)
- [Curiosity](https://en.wikipedia.org/wiki/Curiosity_(rover))
- [Waymo](https://waymo.com)

1. What problem is the robot solving? Your answer should be focused on the
   problem people had, why it was hard, and why it was important to solve. 

2. What sensors does the robot use to solve the problem? What does it need to
   know about its environment, and how does it find out?

3. What actuators does the robot use to solve the problem? What does it need to
   alter about its environment, and how does it do so?

4. How well does the robot work? How is its performance quantitatively measured
   or qualitatively evaluated?

5. How does the robot fail? What happens when it fails?

6. How much does the robot cost? If you find a price, cite your source. If you
   cannot find a direct source, give an estimate and some reasoning.

## Law of Leaky Abstractions (20 points)

*Write your answers in the corresponding section of `answers.txt`*

Read [The Law of Leaky
Abstractions](https://www.joelonsoftware.com/2002/11/11/the-law-of-leaky-abstractions/).
How might this be especially relevant to robotics? Make sure you address:
- How might implemented systems not be true to their modeled behavior?
- How can we use abstractions in light of these challenges?

## Handin

If you do not have a Github account, please create one at [this
link](https://github.com). We will be using git repos throughout the course for
versioning, moving code around, and submitting assignments.

Once you have a github account, click on [this
link](https://classroom.github.com/a/Yc9ObA6D) to join our Github classroom.
This should ask you to select your name from the list and create a repository
for you. Clone the directory to your computer

`git clone https://github.com/h2r/introduction-yourGithubName.git`

This will create a new folder. Before you submit your assignment, your folder
should contain

* collaboration_policy.pdf
* safety_policy.pdf
* answers.txt

Commit and push your changes before the assignemt is due. This will allow us to
access the files you pushed to Github and grade them accordingly. If you commit
and push after the assignment deadline, we will use your latest commit as your
final submission, and you will be marked late.

```
cd introduction-yourGitHubName
git add collaboration_policy.pdf safety_policy.pdf answers.txt
git commit -a -m 'some commit message. maybe handin, maybe update'
git push
```

Note that assignments will be graded anonymously, so don't put your name or any
other identifying information on the files you hand in. 

If your name is not in the list of names, please email
cs1951rheadtas@lists.brown.edu and we will make sure your name is added to the
list.
