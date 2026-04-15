# batbot_bringup

The batbot_bringup contains some GUI applications for interfacing with Sonar and Motor boards from a host computer for data collection or control.

- `bb_gui` - The main batbot7 GUI application. Contains controls for motors and sonar and ability to upload chirps to sonar board. Able to save microphone data.
- `pinnae` - GUI application for controlling tendons.
- `receive` - GUI application for viewing received Sonar microphone data.

## Setup and Installation

Build the python GUI application by navigating to the `batbot_bringup` directory and running:
```
python setup.py build
pip install .
```