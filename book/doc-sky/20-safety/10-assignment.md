# Assignment {#safety-assignment status=ready}

The goal of this assignment is to ask you to think critically about
how to ensure robots are operated safely, and to devise guidelines for
operating your robot safely.

## Problem: OSHAA Safety Analysis (50 points) 

Read the OSHA Technical Manual on <a
href="https://www.osha.gov/dts/osta/otm/otm_iv/otm_iv_4.html">Industrial
Robots and Robot System Safety</a>.  Perform a hazard analysis for the
drone based on the OSHA guidelines. Summarize the information given in
the <a href="../../projects/build/build.html">build project</a>.  Try
to be complete and creative!  We will be using your answers in our own
hazard analysis, and in creating safety guidelines, and we will
discuss in class. Think about how you will program the robot, how you
will start it up, and what procedures you need.  We have also
published a proposed checklist in the <a
href="../../safety.html">Safety page</a>.  We will be mining your
answers for additional suggestions as well, and will publish merged
guidelines on the safety page!  We are looking for three to five
sentences to answer each question.

  1.  What tasks will the robot be programmed to perform? 
  1.  What are the startup, command, or programming procedures? ()
  1.  What environmental conditions are relevant?
  1.  What are location/installation requirements to fly the drone?
  1.  What are possible human errors?
  1.  What maintenance is necessary?
  1.  What are possible robot and system malfunctions?
  1.  What is the normal mode of operation?


### Answers {.hidden}

Please see the feedback given as part of your grade.  We have given
our answers below. 
<ol>
    <li><b>What tasks will the robot be programmed to perform?</b>  <p> The robot will be programmed to fly on trajectories in indoor and outdoor environments.  It will be flown for educational purposes, demonstrations, and to capture video.</p></li>
    <li><b>What are the startup, command, or programming procedures?</b>  <p>To start the drone, follow the safety checklist.  You will use a laptop base station to program the drone, connecting over SSH.   You will also use a Javscript web page for programming.  If you wish you may also install ROS and connect to it using those tools.</p></li>
    <li> <b>What environmental conditions are relevant?</b> <p>Relevant environmental conditions include the wind/air flow around the drone, people who are nearby who may be injured or obstacles, and obstacles the drone may fly into.  Additionally drone needs to be flying above a surface visible to the IR sensor and above a textured, non-uniform surface in order to localize itself with the camera.  Finally the drone needs to connect to a base station over wifi, so the environment must not have too much RF interference.  If the drone is flying outside it must also comply with all FAA regulations and flight restrictions.</p></li>
    <li>  <b>What are location/installation requirements to fly the drone?</b> <p> The drone should be flown inside in an environment that is robust to collisions.  There needs to be a wifi connection.  If flown outside it must comply with FAA rules. </p></li>
    <li> <b>What are possible human errors?</b> <p>  There are lots of possibilities.  The person might connect to the wrong drone, and spin up a drone that someone else is working on.  The person could get the frame of reference wrong and fly the drone forward when they mean to fly it backward.  The person could fly over a non-textured surface or reflective surface, and the drone could fail to localize or find its height accurately.  The person could fly the drone too high, leading to less accuracy at estimating height and velocity.  The person could fail to hit the kill switch correctly in an out of control situation.  And more...  </p> </li>
    <li> <b>What maintenance is necessary?</b>  <p> Before each flight, inspect the drone to make sure all parts are secure and in order.  Make sure no wires are obstructing the camera and IR sensor. Make sure nothing can collide with the props.  Make sure the battery is charged.  Other items are on our safety checklist on the <a href="safety.html">safety</a> page.</p> </li>
    <li> <b>What are possible robot and system malfunctions?</b> 

<p><ul>
<li>Run out of battery</li>
<li>Collide with something</li>
<li>Network drop-out</li>
<li>Prop hits part of the drone, especially a wire</li>
<li>Vibration works something loose and a part flies off</li>
<li>A wire comes unplugged </li>
<li>Crash causes a battery short that burns out components</li>
<li>Parts overheat (PI or power distribution board)</li>
</ul>
</p>  </li>
    <li> <b>What is the normal mode of operation?</b> 
 <p></p>

</li>
</ol>



## Problem: FAA Rules  (10 points)

Read the FAA page on <a href="https://www.faa.gov/uas/">Unmanned
Aircraft Systems</a> We will be flying in accordance with the FAA
Special Rule for Model Aircraft.  Do you need to register your drone?
When do you need to report an accident to the FAA?


### Answers {.hidden}

You do not need to register your drone because you are flying for
recreational use.  You need to <a
href="https://www.faa.gov/uas/faqs/#air">report an accident to the
FAA</a> "within 10 days if it results in at least serious injury to
any person or any loss of consciousness, or if it causes damage to any
property (other than the UAS) in excess of $500 to repair or replace
the property (whichever is lower)."



## Problem: Case Study (10 points)

You would like to fly your drone at home over Thanksgiving break.
What are the risks?  What should you do to plan?  What safety
precautions should you take before you fly?


### Answers {.hidden}
Plan a safe place to fly - think about things that your drone might
hit and damage.  Make sure all observers/bystanders have safety
equipment or stand and adequate distance away.  Test it first,
especially if you allow bystanders to observe without safety
equipment.  Consider how you will connect to the drone - using its own
Wifi or the household wifi.    Make sure the wifi connection is strong so that you can disarm the drone if necessary.



## Problem: Case Study (10 points)

You would like to fly your drone in the lobby of the CIT.  What are
the risks?  What should you do before you fly? 

### Answers {.hidden}

Find out if you are allowed to fly in this space.  (As
I was writing the answer to this question, I realized I didn't know
and just emailed Tom Doeppner and Peter Haas.)  Think about things
that your drone might hit and damage.  Make sure all
observers/bystanders have safety equipment or stand and adequate
distance away.  Fly late at night when fewer people are around.  Test
it first in a nearby area such as CIT 115, to make sure there are no
wifi or control issues.  Consider whether you will use RLAB or the
drone's own wifi and make sure you test the connectivity first.



## Problem: NTSA Report  (20 points)

Read the <a
href="https://www.ntsb.gov/investigations/AccidentReports/Pages/AAR1005.aspx">NTSB accident
report</a> which we discussed in lecture, about the Midair Collision
Over Hudson River. Click the "Related Report" link to download the
54-page PDF.

What can you learn from this incident about safe flying?  What
additional precautions does it suggest for safely operating your
drone?

### Answers {.hidden}

The fact that they checked whether each pilot was under the influence
of drugs or fatigued suggested that one should also do a
self-assessment before flying a drone.

The air traffic controller's nonpertinent phone call suggested one
should be careful of distraction and focus when flying.

The way they also held the supervisor responsible suggested that
everyone is responsible for safety.



## Handin

If you do not have a Github account, please create one at [this link](https://github.com). We will be using git repos throughout the course for versioning, moving code around, and submitting assignments.

Once you have a github account, click on [this link](https://classroom.github.com/a/Yc9ObA6D) to join our Github classroom. This should ask you to select your name from the list and create a repository for you. Clone the directory to your computer

` git clone https://github.com/h2r/safety-yourGithubName.git `

This will create a new folder. Before you submit your assignment, your folder should contain

* safety.pdf
* README.md

Commit and push your changes before the assignemt is due. This will allow us to access the files you pushed to Github and grade them accordingly. If you commit and push after the assignment deadline, we will use your latest commit as your final submission, and you will be marked late.

```
cd safety-yourGitHubName
git add safety.pdf README.md
git commit -a -m 'some commit message. maybe handin, maybe update'
git push
```

Note that assignments will be graded anonymously, so please don't put your name or any other identifying information on the files you hand in. 

If your name is not in the list of names, please email cs1951rheadtas@lists.brown.edu and we will make sure your name is added to the list.
