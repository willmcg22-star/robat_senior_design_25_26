.. _testing-guide:

Testing Guide
==================

Embedded Testing Procedure
##############################

The embedded software applications utilize unit testing through PlatformIO to ensure that all the code is working.
The embedded software has two types of tests: hardware tests and native tests.
Native tests are tests for algorithms or any other code that isn't dependent on the hardware.
Hardware tests, of course, rely on hardware.
Within the embedded projects, all tests are located in the ``test`` folder which should have ``embedded`` (AKA hardware) and ``native`` tests.
These folders indicate what kind of tests they should contain and all unit tests are written using the Unity testing framework.

Both hardware and native tests are used with GitHub Actions to perform continuous integration (CI), which ensures that the main branch of the GitHub always has working and tested code, by running native tests everytime pull requests are made.
Hardware testing is a bit more complicated but is still possible with to run with GitHub actions through PlatformIO's remote features.

Even though tests are run automatically, it is still useful to run tests on your own machine. The following guide outlines the testing process:

1. Write your tests
    Be sure to place your tests in the correct folder depending on the type of test. The Unity testing framework is utilized for tests.

2. Run the tests
    To run your tests, use PlatformIO's testing tools located on the side bar:

    (TODO: Add a picture)

    A menu will pop out and you can select specific tests to run.
    
    Alternatively, sometimes it works better to run tests through the PlatformIO CLI. Open up a PlatformIO CLI as shown below:

    To run a test run the following command:
    ::

        pio test -e [ENVIRONMENT_TO_TEST] 

    The test command is described in more detail here: https://docs.platformio.org/en/latest/core/userguide/cmd_test.html

Common Testing Issues
^^^^^^^^^^^^^^^^^^^^^^^^^

The most common testing issues we've faced have to do with uploading tests to the microcontroller during hardware testing.
This can usually by resolved by forcing the microcontroller into upload mode.
For some reason, a lot of port changes occur during testing as well (mostly on Windows), so ensure that you're tests are being uploaded to the write microcontroller.

If you ever write tests that don't return after a while, something definitely went wrong and you should take a look at your code.

Hardware-In-the-Loop 
^^^^^^^^^^^^^^^^^^^^^^
As mentioned above, it is possible to run automatic CI hardware tests using PlatformIO remotes.
The lab doesn't currently have this capability, but if you would like to set this up, we've outlined the steps here:

1. Set up a testing server
    Hook up a computer (a Raspberry Pi server might be good for this) hooked up to the microcontroller
2. Set up the PlatformIO remote
    Install the PlatformIO CLI on the testing server and `authenticate it with PlatformIO remotes <https://docs.platformio.org/en/latest/core/userguide/account/cmd_login.html>`_. A PlatformIO account will be needed for this. 
    Then run the command:
    ::

        pio remote agent start

    You can validate the connection by logging in to PlatformIO the same way on your computer and running the command:
    ::

        pio remote agent list

    You should see an entry for the agent running on the testing server.

3. Run a remote test
    From here you can run ``pio remote test`` using the same arguments we passed in the testing procedure above.
4. Create a GitHub action
    Create a GitHub action to run this command on every pull request or commit as needed. It is recommended to use GitHub secrets for managing PlatformIO account login.