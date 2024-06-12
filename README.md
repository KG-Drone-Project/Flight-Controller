# Flight-Controller
> Rust-based flight controller for an STM32-based quadcopter.

[![Language](https://img.shields.io/badge/Language-Rust-orange.svg)](https://www.rust-lang.org/)
[![Platform](https://img.shields.io/badge/Platform-STM32-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)

## Introduction

This repository contains the code for my Rust-based flight controller for my quadcopter.

## Hardware & Components 

- STM32F411CEU6 Microcontroller
- LSM6DSOX IMU 6-axis
- LITTLEBEE 30A Electronic Speed Controller (4x)
- ReadyToSky Motors (4x)
- LiPo Battery 

## RTIC

[RTIC](https://rtic.rs/2/book/en/) is a real-time framework written in Rust. It makes use of the microcontroller's hardware to schedule tasks instead of a software kernel. This allows for deterministic behavior and better security than a traditional real-time OS. I employ the RTIC framework to schedule the following three tasks.

### Tasks

#### Polling the USART
I have a task to poll the USART hardware for new data from the FlySky Receiver. In the same task, I verify the checksum, process the data, and enqueue it so that other tasks may dequeue the FlySky data.

#### Kalman Filter
I have another task to employ a Kalman Filter that will fuse the IMU data. My IMU is composed of an accelerometer and gyroscope. Using the Kalman Filter will allow me to get a better representation of the roll and pitch of the drone from the IMU. I enqueue the roll and pitch so that other tasks may make use of these values.

#### Flight Controller
This task employs a PID controller to send the proper throttle signals to the ESC. This task will dequeue the FlySky queue and the Kalman Filter queue to get the desired roll/pitch and actual roll/pitch, respectively. The PID will adjust the throttle to minimize the difference between the desired and actual roll/pitch values. 

## Current Issues

The vibrations on the drone from the motors cause the accelerometer's data on the IMU to get very noisy. This ultimately throws off the values from the Kalman Filter, as they become very sporadic and unreliable. This disallows the PID from correcting the error. I either plan on tuning the Kalman Filter, and if that does not work, I will switch to another Kalman Filter implementation.
