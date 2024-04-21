# ECEN5623-FinalProject

## Overview
This project implements a simple object detection system using an HC-SR04 ultrasonic ranging sensor and a Rasberry Pi 4.
It also utilizes `openCV` and a Logitech c270 webcam to livestream the object detection range to a monitor.
Object range is indicated by the "alarm" LED that blinks periodically based on the detected range.

### Hardware
The link below details the circuit for the HC-SR04 sensor used in this project to interface with the RPi4.
https://thepihut.com/blogs/raspberry-pi-tutorials/hc-sr04-ultrasonic-range-sensor-on-the-raspberry-pi

Note the HC-SR04 outputs logic at 5V, and the RPi4 can only accept 3.3V inputs, so a simple voltage divider is implemented via 2 resistors.

The Alarm LED is connected to RPi4 GPIO26 (hardware pin 37), then to ground via a 470ohm resistor. See the RPi4 Documentation for reference.
https://www.raspberrypi.com/documentation/computers/raspberry-pi.html