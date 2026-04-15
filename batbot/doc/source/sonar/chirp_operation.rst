.. _chirp-operation:

###############
Chirp Operation
###############

**********************
Single Chirp Operation
**********************

Single chirp operation is the most basic method of operation for the sonar system, and therefore also the simplest to use. It should be used in situations where a large quantity of chirps are not needed, such as when testing the system.
Note that because of the nature of sonar systems to pcik up environment noise and other distractions, the data is inherently variable. It may be necessary to run multiple chirps and analyze multiple data sets if precise data is required.
Single chirp operation is peformed through the use of the run_chirp.py script, which can be found in the fieldbot repository. More information on this script can be found in the :ref:`sonar-software` section.

To run a single chirp, follow these steps:
1. Ensure that the ItsyBitsy M4 is connected to your computer and that the run_chirp.py script is running in VSCode.
2. Ensure that the amplifier is powered on and that the system is properly set up, as detailed in the :ref:`setup-and-assembly` section.
3. Open the terminal in VSCode and run the command ``python run_chirp.py [output_name] [time_offset] [frequency_low] [frequency_high]``, where:
   - ``output_name`` is the name of the output file to save the data to.
   - ``time_offset`` is the time offset in seconds to apply to the data (optional, default is 0).
   - ``frequency_low`` is the low frequency of the chirp in Hz (default is 30000).
   - ``frequency_high`` is the high frequency of the chirp in Hz (default is 100000).

**************************
Continuous Chirp Operation
**************************

TODO: Explain the process of continuous chirp operation once Elias has finished the GUI and continuous chirp files.

*****************
Component Testing
*****************

It is the nature of lab work and engineering in general that components will fail, often without warning. It is important to be able to test individual components of the sonar system
to ensure that they are working properly and to be able to diagnose and identify issues when they arise. Most components can be tested individually, as detailed in this section. It is very
rare that multiple components will fail at once, so much of the testing will be done using other components as part of the test. Refer to this section when diagnosing issues such as no chirp appearing
in the spectrogram, weak chirps, or suspected individual component failures. 

Microphone and Transducer
^^^^^^^^^^^^^^^^^^^^^^^^^

Testing of the microphone and transducer can be done using the SDG 1032X function generator within the lab. Set the function generator to output a sine wave at your desired frequency (50kHz is a good baseline),
and connect the output of the function generator to the JP4 pins, removing the jumper pins which reside on these pins. Run a single chirp, and you should see a strong signal at the frequency you set on the function generator.
It should appear as a horizontal line on the spectrogram. If this line appears, the microphone and transducer are both working properly, and the amplifier is very likely to also be working properly. If this line does not appear,
first test the microphone by disconnecting the transducer wires from the system, and playing a sound at a known frequency (Youtube often has videos playing a specific frequency), being sure to adjust your spectrogram bounds to include your chosen frequency.
If you see a line at the frequency of the sound you played, the microphone is working properly. If you do not see a line, the microphone is likely faulty and should be replaced. If the microphone is working properly and the original test did not show a line,
the transducer or amplifier is likely faulty. To test the transducer, simply swap it out for a known working transducer (potentially multiple others if needed) and repeat the original test. If a line still does not appear, the amplifer is likely the issue.

.. figure:: ../img/testing_line.PNG

   *An example of the aforementioned line in a spectrogram*

Amplifier
^^^^^^^^^

The easiest way to tell if the amplifier is faulty is to observe the power draw from the power supply. If the draw is <0.05A and/or the fan on top is not spinning, the amplifier may be faulty. If it is drawing >0.15A, the amplifier is likely working properly. 
Refer to the :ref:`setup-and-assembly` section for more details on amplifier positioning and power draw, as it is very common for a working amplifier to be misdiagnosed as faulty due to poor contact with the PCB. If the amplifier is still not drawing enough current
after repositioning efforts, switch it out for a known working amplifier and see if the current draw increases to an acceptable level. If it does, the original amplifier is likely faulty. If it does not, the issue is likely with the PCB board.

ItsyBitsy M4
^^^^^^^^^^^^

The easiest way to test the ItsyBitsy is by using standard software debugging practices, namely using print statements after key lines of code to ensure that the code is running properly. If you are able to see print statements in the serial monitor, the ItsyBitsy is likely working properly.
For example, helpful places to include print statements are after serial read or writes commands, since these will not execute if there is a communication issue. If you suspect hardware issues with the ItsyBitsy, you can swap it out for a known working ItsyBitsy and see if the issue persists
with the same software. It is most helpful to use a version of the code that is known to be working properly, such as the version in the main branch of the fieldbot repository. If the issue persists with a known working ItsyBitsy and known working code, the issue is likely with another component.

PCB Board
^^^^^^^^^

There is no easy way to test the PCB board itself, but if all other components have been tested and are working properly, the board is likely the issue. One way to spot a faulty board is to use a known amplifier and ensure that it is drawing enough current.
If it is not, the board may be faulty. Additionally, if the board is not powering on at all (no red LED), the board may be faulty. If you have tested all the indivual components and you are still seeing a spectrogram with only passthrough and no chirp,
the board is most likely the issue. Board issues are often difficult to diagnose, and may require a new board to be soldered, as the issues may be internal and not easily visible. It is good practice to have multiple boards on hand in case of a board failure.
Boards can be ordered from JLCPCB, and the design files can be found in the Fusion drive. Refer to this `tutorial <https://docs.google.com/document/d/1YvDp_zSqT3rgwtFb2dRUX3VRMfaroaLDDXuvnbMSMGw/edit?usp=drive_link>`__. Component lists can be found in the Fusion drive as well.