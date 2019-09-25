# Assignment {#networking-ros-assignment status=ready}

This assignment is comprised of three parts: Introduction to Linux (Part 1), Networking (Part 2), and ROS (Part 3). Please complete all parts of this assignment. 

# Part 1: Introduction to Linux

## Background Information

When you enter a command in a shell, it executes a program. These programs read
from a stream, known as "standard input" and write to two output streams,
"standard output" and "standard error". When you `print` in python, it writes
its output to standard output. In another language, such as C, you use other
functions, such as `printf` to write to standard output.

In addition to writing to standard output, a program can read from standard
input. The program `cat`, short for concatenate, reads from standard input
and writes the result to standard output.

## Standard Output (10 points)

1. Write a python program that prints "Hello world" to standard output. *Save
   the program as `hello1.py` and submit it.*

2. Write a python program that prints "Hello world" to standard output using
   `sys.stdout`. *Save the program as `hello2.py` and submit it.*

3. Write a bash script that prints "Hello World" to standard output. *Save the
   script as `hello.sh` and submit it.*

## Standard Input (10 points)

*Write answers to questions 1-2 in`shell.pdf`. Submit this file.*

1. Run `cat` with no arguments. Why does `cat` seem like it is hanging?

2. When you run `cat`, type a message into your terminal, and press
   `Control-D`. Describe what `cat` does. Make sure to include which streams
   are being used, and for what purpose.

3. Write a python program `my_cat.py` that reads a message from standard input
   and prints to standard output, just as `cat` does. *Submit this file.*

## Pipes (20 points)

Pipes are used to redirect standard input, standard output, and standard error.
First, `>` is used to redirect standard output to a file. For example, `echo "Hello World" > test.txt` will write the string `Hello World` to `test.txt`. *Write answers to questions 1-4 in `shell.pdf`. Submit this file.*

1. Create files `one.txt`, `two.txt` and `three.txt` that contain the strings
   `1`, `2`, and `3`, respectively using `echo` and output redirect. 
   
2. By convention, almost all shell programs read input from standard input, and
   write their output to standard output. Any error messages are printed to
   standard error. You can chain shell programs together by using `|`. For
   example, the program `ls` writes the contents of a directory to standard
   output. The program `sort` reads from standard input, sorts what
   it reads, and writes the sorted content to standard output. So you can use
   `ls | sort` to print out a sorted directory list. Read the man page for sort
   (`man sort`) to learn how to sort in reverse order. What is the bash script (using `|`) that prints the contents of a directory in reverse alphabetical order? 

3. Use `cat`, `|` and `echo` to print `hello world.` Do not write to any files
   and use both commands one time.

4. This is not the simplest way to print hello world. Can you suggest
   a simpler way? (We asked you to do it the more complicated way to practice
   with pipes.) 
5. Write a python script that reads from standard input, sorts lines in reverse
      alphabetical order, and prints the result. It should behave like `sort -r`.
      *Submit your script in a file called `my_reverse_sort.py`. Do not submit this script in `shell.pdf`*

## Standard Error (10 points)

In addition to standard input and standard output, there is a third stream,
standard error. If there is an error in a chain of pipes, it will be printed to
the terminal rather than buried in the input to the next program.

1. Recall that `ls -a | sort > sorted.txt` puts all the names of files in
   a directory sorted in alphabetical order into the file `sorted.txt`. If you
   modify the command to be `ls -a -hippo | sort > sorted.txt`, what text is in
   `sorted.txt`, what is outputted as standard error, and why? *Answer this question in `shell.pdf`. Submit this file.*
2. Create a python script that, in addition printing sorted inputs to standard
   out, prints status reports to standard error. Use it to sort `ls -a` instead
   of `sort`. *Submit the file containing the script as `my_sort_status.py`.*

# Part 2: Networking

## Netcat (20 points)

The command `nc` is short for "netcat" and is similar to `cat` but works over
network connections. It reads from standard input and writes its contents not
to standard output, but to a specified server. *Write your answers in the
corresponding sections of `networking.pdf`.*

1. Point `nc` to google.com as follows: `nc www.google.com 80` When you first
   connect, it will be silent. Then type any arbitrary text and press enter.
   What is the error number?

2. Now type some valid http into nc: `GET / HTTP/1.1`. What is the output?

3. Now use `nc` to make a server. In one window, type `nc -l 12345`. This
   will cause `nc` to listen on port 12345. In another terminal on the same
   machine, type `nc localhost 12345`. You can type a message in one window
   and it will appear in the other window (and vice versa). This trick can be
   very useful to test basic internet connectivity - if the client and server
   can send packets at all. *No answer is required for this question.*

4. By convention, `roscore` listens on port 11311. Try using `nc` to connect to
   port 11311 on a machine where `roscore` is running, such as the Pi on your
   drone. What protocol is roscore using to communicate?


## Talking to Your Robot (10 points)

So far, this assignment has required access to `localhost`, the local machine
you are connected to, and `google.com`.

Most commonly, the base station and robot are connected over TCP/IP to the same
local network. Then you can look up your machine's IP address (`ifconfig` in
Unix; other ways in other OSes), and your robot's IP address, and connect them.
How can you find your robot's IP address? Well it's a chicken-and-egg problem.
If you knew the IP address, you can connect to the robot and run `ifconfig` and
find the IP address, but you don't know the IP address.

What to do? There are several solutions. *Write the answers to the following
questions in `networking.pdf`.*

1. Brainstorm how you can solve the chicken-and-egg program to connect to
your robot. List three different solutions. 

## Look Ma, No Internet! (10 points)

But what about if there *is* no public internet connection? What if you want to
fly your drone in the wilderness? Well, there does exist cellular modems and
satellite connections, but you can also tell your drone to act as a Wifi
Hotspot. It can create a network and run a DHCP server. You can configure this
on your drone using the file `/etc/hostapd/hostapd.conf`. Then you can connect
your laptop's base station using the SSID and passphrase specified in that
file, and connect to the drone.

Alternatively you can set up your laptop as the Wifi base station and configure
the drone to connect to its network. The details will vary depending on your
laptop OS and settings. 

Your Pi is configured to be a Wireless AP Master by default. Connect to it with
your base station. *Write the answers to the following questions in `networking.pdf`.*

1. Which machine is acting as the DHCP server?
2. What is the Pi's IP address? What is yours?
3. What other devices on the network could be Wireless AP Master? What other devices on the network could act as the DHCP server? 
4. Describe three network configurations for a network allowing a basestation and PiDrone to communicate with each other. 

# Part 3: ROS

## Creating a Publisher and Subscriber (50 points)

*Answer these questions in `ros.pdf`  and submit the ROS package you create.*

1. Read [understanding nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes).
2. Start the `screen` session we use to fly the drone. Use `rosnode list` to display what nodes are running when you start the screen. If you wish, take a look at the [software architecture diagram](https://docs-brown.duckietown.org/opmanual_sky/out/software_architecture_assignment.html#sec:software-architecture-assignment) and look at all of the blue ROS topics to gain a visual understanding of all of the nodes that are running. Once again, do not worry about understanding everything now, or knowing what each topic is used for- you will learn this through experience as the course progresses. *No answer is required for this question*
3. Use `rosnode info` to find out more about as many nodes as you'd like. What topics does
   `/pidrone/infrared` publish?
4. Do the ROS tutorial to [create a package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage). Name your package `ros_assignment_pkg`.
5. Do the [building packages](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) tutorial.

6. Follow the [ROS publisher/subscriber tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) using the workspace and package you created above. *Hand in the entire package.*
7. Start the `screen` session we use to fly the drone. Use `rostopic echo` and `rostopic hz` to examine the results of various topics. What is the rate at which we are publishing the infrared range reading?

## Messages (5 points)

*Make all modifications in your ROS package from Problem 1 and hand in the package*

1. Read [Creating a ROS msg](https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv). You do not need to read the section on services.
2. In your package from question 1, create a ROS message called `MyMessage`
   with a field for a `string`, called `name`, and a field for an array of
   `float64`, called `contents`. Edit files such as `CMakeLists.txt` to ensure
   your message is compiled and available for use. *Make these modifications in the package from problem 1 and hand it in.*

## Reading the IR Sensor (15 points)

1. Write a ROS subscriber on your drone to read the values from the infrared
   sensor topic and print them to `stdout`. *Name the file `my_echo.py` and
   submit it.*
2. Write a second ROS subscriber that listens to the infrared sensor topic and
   calculates the mean and variance over a ten second window using
   [NumPy](https://jakevdp.github.io/PythonDataScienceHandbook/02.02-the-basics-of-numpy-arrays.html). Print these values to `stdout`. *Name the file `mean_and_variance.py` and submit it.*

## Handin

When you are done, use [this link](https://classroom.github.com/a/NAGxz1QJ) to create your assignment Github Repo.

- `hello1.py`, `hello2.py`, `hello.sh`, `my_cat.py`, `my_reverse_sort.py`, `my_sort_status.py`, `my_echo.py`, `mean_and_variance.py`
- `shell.pdf`, `networking.pdf`, `ros.pdf`	
- `ros_assignment_pkg`
