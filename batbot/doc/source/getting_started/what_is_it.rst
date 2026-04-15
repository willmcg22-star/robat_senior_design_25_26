.. _what-it-is:

What is the Batbot Project?
===========================

The batbot project is a research project based in the Center for Bio-Inspired Science and Technology Laboratory at Virginia Tech.
Sponsored by the Naval Engineering Education Consortium (NEEC) and the Naval Underwater Warfare Center (NUWC) Newport, we 
are a team of graduate and undergraduate students working under Dr. Muller at Virginia Tech to develop 
robotic systems that mimic the movement and echolocation abilities of bats as they navigate through complex foliage.
This documentation focuses on the software applications related to the echolocation aspect of the project. Current 
BIST research efforts focus on applying soft robotics and machine learning techniques to achieve a biomimetic 
sonar system.

System Overview
-----------------

The batbot is a robotic platform that is intended to be used for reinforcement learning experiments. The platform consists of two major subsystems:
the Sonar System and the Tendon Actuation System.

The Sonar System
^^^^^^^^^^^^^^^^
The sonar system is designed to mimic the echolocation abilities of bats. The sonar consists of a microphone housed in a silicon molded pinnae based on bat ear morphology
and a speaker housed within a 3D printed nose-leaf. These components work in tandem to create a "chirp."
A chirp consists of an echo, a short burst of ultrasonic sound emitted by the sonar system, and a listen, the reflected sound received by the sonar system.


The Tendon Actuation System
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The tendon actuation system is designed to mimic the movement of bat ears.
The idea is that by shaping the pinnae during a chirp, we can optimize the sonar system's ability to detect and interpret echoes in various scenarios.
A series of cable-driven tendons are used to deform the pinnae and a microcontroller controls the motors that actuate these cables.

The microcontroller is programmed to perform relative position control of the motors, and a serial interface is exposed to allow users to control the system from a computer.

Current Project Status
-----------------------
The sonar system and tendon actuation system are currently in development.