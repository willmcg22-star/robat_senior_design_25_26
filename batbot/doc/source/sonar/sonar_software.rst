.. _sonar-software:

####################
Sonar Software Guide
####################

The software for the sonar system is mainly housed within the `fieldbot <https://github.com/BIST-Research/fieldbot>`__ repository within the BIST Research Github.
The sonar software provides the ability to send and recieve sonar "chirps", and visualize the 
data in a spectrogram and waveform display. A chirp is a frequency sweep from high to low frequency over a small period of time,
which in our lab is designed to replicate the echolocation abilities of bats. 

In order to use the sonar software, the following steps must be followed:

- Install PlatformIO IDE extension in VSCode and ensure that you are able to establish connection with the ItsyBitsy M4 microcontroller.
- This often will not work at first, and may require additional steps. Refer to the `PlatformIO documentation <https://docs.platformio.org/en/latest/integration/ide/vscode.html#ide-vscode>`__ for more details. 
- Install Python to your computer from `python.org <https://www.python.org/downloads/>`__ and ensure that it is added to your system PATH.
- While in the fieldbot repository, run the following command in the terminal to install the required Python packages: ``pip install -r requirements.txt``

Once these steps have been taken, you should be able to run the sonar software. Operation instructions can be found in
:ref:`chirp-operation`.

*********************
Sonar System Overview
*********************

Sending A Chirp
======================

Sending a chirp starts with a software command to the ItsyBitsy. Voltage is sent from the ItsyBitsy's DAC pin through the PCB to the amplifier, which amplifies the signal and sends it to the
transducer. The transducer then emits the chirp, which travels through the air until it hits an object and reflects back to the microphone, which transmits the signal through the ItsyBitsy's ADC pin.
Once the ItsyBitsy recieves the signal, it processes the data and displays it in a spectrogram and waveform view. The data is also saved to a file for later analysis.


Recieiving A Chirp
=====================

TODO: Explain the process of recieving a chirp

Processing Chirp Data
=======================

TODO: Explain how the data is processed, including filtering, FFT, etc.


**************
Relevant Files
**************

TODO: Include a brief documentation for the most important files.

run_chirp.py
--------------

TODO: Explain the purpose of this file, and a brief overview of how it works.

ml_main.cpp
-------------

TODO: Explain the purpose of this file, and a brief overview of how it works.

run_chirps.py
--------------

TODO: Explain the purpose of this file, and a brief overview of how it works.


TODO: Add explanations for Elias' continuous chirp files and GUI
