## Duckiesky 

This is the documentation repository for Duckiesky.  It contains
   materials for teaching Introduction to Robotics: Duckiesky.

## Fork this repo

Fork this repo to edit it.


## Running using Docker

First run:

    $ git submodule sync --recursive
    $ git submodule update --init --recursive

# Installing Docker

Then install Docker:

    $ make install-docker-ubuntu16

*Note*: you need to be in group `docker`. The script adds you, but it does not take effect immediately. You need to exit the console and re-enter.

# Compiling

Then compile using:

    $ make compile-docker


# Viewing

Point your web browser to 