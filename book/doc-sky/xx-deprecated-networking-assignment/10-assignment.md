# Assignment {#networking-assignment status=ready}

## Background Information

When you enter a command in a shell, it executes a program. These programs read
from a stream, known as "standard input" and write to two output streams,
"standard output" and "standard error". When you `print` in python, it writes
its output to standard output. In another language, such as C, you use other
functions, such as `printf` to write to standard output.

In addition to writing to standard output, a program can read from standard
input. The program `cat`, short for concatentate, reads from standard input
and writes the result to standard output.

## Standard Output (10 points)

1. Write a python program that prints "Hello world" to standard output. *Save
   the program as `hello1.py` and submit it.*

2. Write a python program that prints "Hello world" to standard output using
   `sys.stdout`. *Save the program as `hello2.py` and submit it.*

3. Write a bash script that prints "Hello World" to standard output. *Save the
   script as `hello.sh` and submit it.*


## Standard Input (10 points)

1. Run `cat` with no arguments. Why does `cat` seem like it is hanging?

2. When you run `cat`, type a message into your terminal, and press
   `Control-D`. Describe what `cat` does. Make sure to include which streams
   are being used, and for what purpose.

3. Write a python program `my_cat.py` that reads a message from standard input
   and prints to standard output, just as `cat` does. *Submit this file.*

## Pipes (20 points)

Pipes are used to redirect standard input, standard output, and standard error.
First, `>` is used to redirect standard output to a file. For example, `echo "Hello World" > test.txt` will write the string `Hello World` to `test.txt`.

1. Create files `one.txt`, `two.txt` and `three.txt` that contain the strings
   `1`, `2`, and `3`, respectively using `echo` and output redirect. *Write the
   commands you used to create these files in the corresponding section of
   `networking.pdf`.*

2. By convention, almost all shell programs read input from standard input, and
   write their output to standard output. Any error messages are printed to
   standard error. You can chain shell programs together by using `|`. For
   example, the program `ls` writes the contents of a directory to standard
   output. The program `sort` reads from standard input, sorts what
   it reads, and writes the sorted content to standard output. So you can use
   `ls | sort` to print out a sorted directory list. Read the man page for sort
   (`man sort`) to learn how to sort in reverse order. What is the bash script (using `|`) that prints the contents of a directory in reverse alphabetical order? *Write the script in the corresponding section of `networking.pdf`.*

3. Use `cat`, `|` and `echo` to print `hello world.` Do not write to any files
   and use both commands one time. *Write your answer in `networking.pdf`.*

4. This is not the simplest way to print hello world. Can you suggest
   a simpler way? (We asked you to do it the more complicated way to practice
   with pipes.) *Write your answer in `networking.pdf`.*

5. Write a python script that reads from standard input, sorts lines in reverse
   alphabetical order, and prints the result. It should behave like `sort -r`.
   *Submit your script in a file called `my_reverse_sort.py`.*

## Standard Error (10 points)

In addition to standard input and standard output, there is a third stream,
standard error. If there is an error in a chain of pipes, it will be printed to
the terminal rather than buried in the input to the next program.

1. Recall that `ls -a | sort > sorted.txt` puts all the names of files in
   a directory sorted in alphabetical order into the file `sorted.txt`. If you
   modify the command to be `ls -a -hippo | sort > sorted.txt`, what text is in
   `sorted.txt`, what is outputted as standard error, and why?

2. Create a python script that, in addition printing sorted inputs to standard
   out, prints status reports to standard error. Use it to sort `ls -a` instead
   of `sort`. *Submit the file containing the script as `my_sort_status.py`.*

## Networking (20 points)

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

5. Another useful tool is `nmap`, which scans through a range of ports (and
   optionally, through a range of IP addresses) and reports information. Run
   `nmap localhost` on your Pi. What ports are open? Look up each port and
   submit what it does.

6. Run nmap with and without the `nc -l 1234` command running from above. What
   is the difference? Why?

7. Run `nmap` with `roscore`. Does `nmap` report `roscore`? Why or why
   not? Use `man nmap` to find command line options for `nmap` that report the
   ROS port 11311.

8. Portscan google.com. List each open port and its purpose.

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


2. How does the internet work? A computer typically uses the DHCP protocol to
   request an IP address from a server that manages the local network. At your
   house, it's likely to be your cable modem or home router. At Brown, CIS
   manages the routers that keep the network up. Once you have an IP address,
   you are on the internet.

   There are serious security concerns with giving direct access to the
   internet, without filtering connections. People could serve SPAM, or they
   could get hacked by bad actors who would use the connection to serve SPAM.
   It's safer to not give people public IP addresses and most organizations
   don't. There aren't a lot of them either - one of the things you pay your
   home ISP for is a public IP address, and you usually only get one.

   To try out DHCP, connect to Brown, Brown_Guest, and RLAB. Report back your
   IP address each time using your operating system. Then connect again. Do you
   get the same address or a different address? *List the IP Addresses for each
   network, and whether or not you get the same address when re-connecting to
   each network in `networking.pdf`.*

3. How can we have more than one device connected to the Internet? The usual
   answer is a protocol called Network Address Translation. This remaps the IP
   address space so that you can have one public IP address that usually
   connects to a router. Then the router has a public (WAN or
   wide-area-network) side with the public IP address) and a private (LAN or
   local-area network) with multiple connections. The IP addresses on the
   private side are not full-fledged IP addresses because they cannot act as
   servers. You can't listen on a port from the private side and connect to it
   from the public internet. However you can do private-to-private connections,
   and many people do, e.g., for games or robots!

   You can also selectively open a connection to the public internet on many
   routers using port forwarding. This can be configured on the router; most
   routers offer a web-based API to configure these kinds of remappings. You
   can say port `11311` on the public side maps to a particular IP address and
   port on the private side, for example.

   Under a typical NAT setting, the robot and the base station will typically both
   connect to the router via DHCP to obtain an IP address. Their IP address will
   be in the 192.168.\*.\* range, or the 10.\*.\*.\* range, both by convention
   used for private local networks. The router's public IP address will be
   whatever it is, and both machines will have internet access through NAT.
   However neither machine will be a server to the public internet. But that's
   okay - they only need to be servers to each other. So they can listen on ports
   and server request using their local (192.168 or 10.0.0.\*) IP addresses.

   *Connect to the Brown_Guest, RLAB, and Brown networks. For each network,
   answer the following questions in `networking.pdf`.*

   3.1. What IP address do you have on each network?
   3.2. What is the router's IP?
   3.3. What ports are open on the router?
   3.4. Use `nmap` to identify the machines on each network. How many are
   there?

## Look Ma, No Internet! (10 points)

But what about if there *is* no public internet connection? What if you want to
fly your drone in the wilderness? Well, there does exist cellular modems and
sattellite connections, but you can also tell your drone to act as a Wifi
Hotspot. It can create a network and run a DHCP server. You can configure this
on your drone using the file `/etc/hostapd/hostapd.conf`. Then you can connect
your laptop's base station using the SSID and passphrase specified in that
file, and connect to the drone.

Alternatively you can set up your laptop as the Wifi base station and configure
the drone to connect to its network. The details will vary depending on your
laptop OS and settings.

Your Pi is configured to be a Wireless AP Master by default. Connect to it with
your base station.

1. Which machine is acting as the DHCP server?

2. What is the Pi's IP address? What is yours?

3. What is the ping time between you and the Pi when you are close to the Pi

4. How far away can you get from the Pi before it starts disconnecting?

5. What is the ping time when you are far away from the Pi?


## Environment Variables (30 points)

GNU/Linux uses environment variables to store configuration information about
a variety of things. You can use `env` to view the environment variables in
your shell on the Rasberry Pi. In bash (and most shells), environment variables
are local to your bash session, so they are often set in configuration files
that are run every time your shell starts, such as `.bashrc`.

1. Log into your Rasperry Pi. Use `X=3` to set the value of an environment
   variable named `X` to the value `3`. Use `echo $X` to display the variable.
   Note that you must prepend `$` to the variable name when reading it, but not
   when setting it.

2. Log into your drone again in a separate SSH session. Use `echo $X` to see
   the value of the environment variable `X`. What happens? Does this work? Why
   or why not?

3. Use `env` to see all the environment variables set in your shell. Pick one.
   Research the one that you picked. Describe 1) What program sets the
   environment variable and 2) What the variable controls. For example, the
   `EDITOR` environment variable is set in the `.bashrc` file when you log in.

4. Start screen in one of your SSH sessions. Our `setup.sh` script sets the
   ROS_MASTER_URI and ROS_HOSTNAME or ROS_IP environment variables in your
   session. In a second SSH session in which you have not run screen (so just
   after you log in), assess the value of the environment variables. Are they
   set to the correct values? What is setting ROS_MASTER_URI? What is setting
   ROS_IP or ROS_HOSTNAME? How did you figure this out? (You might find the
   `grep` command useful. Use `man grep` to find out how to use it.)

## Handin

When you are done, use [this link](https://classroom.github.com/a/7YV_5r9E) to create your Networking Github Repo.
Commit and push the relevant files (networking.pdf, and any scripts you wrote
throughout the assignment) to this Github Repo before the deadline.
