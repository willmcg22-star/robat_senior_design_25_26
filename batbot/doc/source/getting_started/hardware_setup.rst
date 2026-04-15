.. _hardware-setup:

Hardware Setup
=====================

Sonar System
-------------

The sonar system consists of a pair of identical sonar modules. A single module is built with the following components:
 
 - 1x SensComp Series 600 Open Face Ultrasonic Transducer (`datasheet <https://github.com/BIST-Research/fieldbot/blob/vehicle/docs/Transducer_Datasheet.pdf>`__)
 - 1x Dodotronic Momonic Analog Microphone (`datasheet <https://github.com/BIST-Research/fieldbot/blob/vehicle/docs/Microphone_datasheet.pdf>`__)
 - 1x Weipu connector for the microphone with hand-twisted wires
 - 1x Adafruit ItsyBitsy M4 microcontroller (`datasheet <https://github.com/BIST-Research/fieldbot/blob/vehicle/docs/ATDAMD51_Datasheet.pdf>`__) (`pinout <http://learn.adafruit.com/introducing-adafruit-itsybitsy-m4/pinouts>`__)
 - 1x BIST custom sonar PCB board
 - 1x Texas Instruments OPA552 high-voltage op-amp (`datasheet <https://www.ti.com/lit/gpn/opa552>`__)
 - 1x PDm200 V9 high performance piezo driver  upon which the op-amp is soldered (`datasheet <https://github.com/BIST-Research/fieldbot/blob/vehicle/docs/PDm200-V9-Datasheet-R5.pdf>`__)
 - JST connectors for the microphone and power connections
 - Screw terminals for the transducer connections
 - 3D printed components for mounting the transducer and ear(optional)

Assembly instructions and more details about the components can be found in the setup and asssembly section of the sonar system documentation:
:ref:`setup-and-assembly`

Tendon Actuation System
-----------------------

Prerequisites
^^^^^^^^^^^^^^^^^^^^

The tendon actuation system consists of a pair of identical bat ear modules. A single module is built with the following components:

- 5x Pololu micro metal gear motors and encoders
- 6-pin JST connectors for the motors and encoders
- 1x Adafruit Grand Central M4 Express microcontroller
- 1x BIST custom motor control PCB shield
- 3D printed components for the ear, tendon pulleys, and motor mounts

Assembly
^^^^^^^^^
The following steps outline the assembly of a single bat ear module (the steps are identical for both ears):

1. Preparing the Motor Shield
    It is recommended to order the BIST custom motor control PCB shield preassembled. If you opt to assemble it 
    by hand, please refer to the design schematic and BOM available in the BIST Fusion 360 repository. Otherwise,
    the preassembled shield can simply be mounted onto the Grand Central M4 Express microcontroller. The shield 
    provides a set of JST connectors for the motors and encoders, as well as two power input connectors for the battery.
    
    .. note::
        Due to pin selection of the current PCB, the motor control software only allows for control of the 5 motor connections highlighted below:
        
        TODO: Add an image

2. Preparing Motor Encoder assembly
    Pololu provides a set of micro metal gear motors with attached encoders. However, the lab also has a set of motors 
    without encoders. If you are using the latter, you will need to manually solder the encoder PCB to the motors like so:

    TODO: Add an image

    The encoder PCBs have 6-pin JST headers which can be hooked up to the motor connectors specified in step 1 via the 6-pin JST connector cables.

3. Preparing the Tendon Pulleys
    TODO: add a description

4. Assembling the Ear
    TODO: add a description