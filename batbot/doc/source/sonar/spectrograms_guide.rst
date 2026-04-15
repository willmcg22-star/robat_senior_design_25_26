.. _spectrograms-guide:


Spectrograms Guide
==================

What is a spectrogram?
----------------------

To measure the distance to an object, we send out a mechanical signal from the transducer away into the space. The signal is a frequency sweep spanning from 30kHz to 100kHz, also known as chirp (this is how it is referred to in-code as well). It bounces off of the objects it encounters, and then comes back towards the microphone^1. Because the chirp comes back continuously, we sample the microphone input over a certain time period. Thus, we can create a diagram of time (x-axis) against frequency (y-axis) against signal intensity (marked by colors; hotter colors are higher signal intensity):

.. figure:: ../img/spectrogram_example.PNG

    *An example of a spectrogram*

We thus use the spectrogram to determine how far the object that the signal bounced off of is.


1: TODO: Add the transducer datasheet & directional characteristic. And fix the references.




What are dBs?
-------------

TODO: Add an explanation of how dBs are a gain of ADC, insert formula of our dBs (ECE classes typically use 20log10 instead of 10log10).


Spectrogram patterns
--------------------

For one reason or another, the spectrograms exhibit a number of unexpected patterns. Ideally, we would receive just one slightly diagonal echo on the object right in front of the sonar setup. However, this comes with a number of caveats:


Passthrough
^^^^^^^^^^^
One of the most notable features of the spectrogram, almost always present is the passthrough. The passthrough is a diagonal line resembling the echo of an object that is captured between 0 and 3 seconds after the chirp has been sent out. We believe its primary origin to be the electromagnetic interference on the PCB, as shielding the cables helps a lot. Sometimes we also observe an alias at twice the frequency of the original passthrough. We are not sure why that is, but we are almost certain that this is the artifact of our signal collection algorithm.

.. figure:: ../img/passthrough_spec_demo.PNG

    *The picture presents a spectrogram in a relative dB scaling mode. After the 50Khz, the signal is practically no stronger than the noise.*


High Frequency Signal Strength
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In most cases, we fail to capture the higher frequencies of our sweep, even though our microcontroller was confirmed to be capable of generating them.


.. figure:: ../img/spectrogram_high_freq_demo.PNG

    *The picture presents a spectrogram in a relative dB scaling mode. After the 50Khz, the signal is practically no stronger than the noise.*

We believe that this issue arises from our amplifier having a steep gain roll-off between 10kHz and 100kHz that the preamplifier inadequately compensates for. Notably, when the shape of the obstacle approaches a flat, perpendicular wall, the signal appears cleaner.

TODO: add the curves for the amplifier


One may also assume that the drop in the strength of high frequencies is contributed to by the air attenuation, and that the signal simply dissipates in air with time. However, attempting to graph the air attenuation coefficient value in response to frequency proves this false:

.. figure:: ../img/air_attenuation_demo.PNG

    *As can be seen, in room conditions we will never lose more than 1.6dB/m of signal strength.*


Vertical Bars
^^^^^^^^^^^^^

Often, we are able to observe a vertical line descending where the highest frequencies of the chirp are. We believe that this is due to the "clicks" that the transducer makes when sending out a chirp. This is supported by the signal existing in the audible spectrum (1-20kHz). In extreme cases, this makes the echo look like a "right triangle" rather than a "diagonal line".

.. figure:: ../img/vertical_bar_spec_demo.PNG

    *The picture presents a vertical bar between roughly 1 kHz and 50kHz at around 4s.*


Horizontal Bars and Notches
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Often the spectrogram exhibits unexpected horizontal bars, usually between 45 and 55kHz but as can be seen on the picture, but as can be seen on the picture, other frequencies are possible too. These bars are the most visible at absolute dB scaling. We are not sure where they come from.


.. figure:: ../img/horizontal_bars_spec_demo.PNG

    *The figure presents the picture of a spectrogram with four distinct visible horizontal lines*


Additionally, these bars are typically coupled with notches (gaps) in echo.

.. figure:: ../img/notch_vs_bar_spec_demo.PNG

    *Same signal with absolute and relative dB scaling. As can be seen, presence of a gap corresponds to the presence of a notch and vise versa*

