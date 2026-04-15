.. _tendon-api-doc:

#############
Tendon API
#############

The Tendon API is a set of Python APIs that allow for easy interaction with the motor control system from a host PC over serial USB connection.
If you reviewed the documentation for the :ref:`tendon-embedded-software`, then you will probably observe that the amount of 
control features for the system comes at the price of complexity. Fortunately, we don't have to expose this complexity to everybody,
and we can abstract away the communication protocol, which is the purpose of these APIs. This page documents how to use the APIs in your own code.
There are two classes in the API and they are documented here.

.. include:: ../batbot_bringup.bb_tendons.rst

Extending the Tendon API
==========================

After creating a new command (as shown in :ref:`tendon-embedded-software`), you will likely want to create a new API function for your command. 
This is as simple as adding a new function to the :class:`~batbot_bringup.bb_tendons.TendonHardwareInterface` class.
You should utilize the class's ``th`` instance variable for automatic parsing and packing of your command packet.

Possible Improvements
=======================

- More graceful error handling on the Python API (use exceptions instead of returns)
- Implement all the functions created in the embedded side
- Alternate communication protocols (SPI)