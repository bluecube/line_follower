Line follower v2
================

Currently work in progress! See [master branch](https://github.com/bluecube/line_follower/) for the most useable version so far.

Changes for version 2:
----------------------
- Motors with encoders
  - Previous version had problems with unpredictable acceleration, since the motors are fast and relatively under-torqued for the robot.
- Bluetooth support with ESP32
- Removable front armor, allowing the robot to take place in sumo competitions as well
- Visible light line sensor, hopefully simplifying debugging
- Whole deck is a (stop) button now
  - improvement over the hard to press wobbly button from v1.
- Enough space on the deck to place race start number sticker.

This is a robot I made to tinker with and to compete in [Robotic Day 2019](https://roboticday.org/) (Actually the original goal was Robotic Day 2016, but stuff got delayed :) )

The design is an attempt to make a line follower with minimum effort on the hardware side and maximum versatility on the software side.

The robot body consists of a 8cm x 10cm double sided PCB, two geared N20 DC motors with Pololu wheels and a few 3D printed bits.
The most important electronic components are Teensy 3.2 development board for computing power and line sensor inspired by [Ondřej Staněk's PocketBot](http://www.ostan.cz/pocketBot/), as a bonus the robot also carries an IR range sensor and MPU 6050 breakout board (accelerometer and gyro).

This repository should contain everything necessary to build and run the robot in the competition.
Let me know if you decide to build it!

![Render of the robot](https://github.com/bluecube/line_follower/raw/v2/fusion360/output-exports/render.png)
