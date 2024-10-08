# flight-controller-using-teensy-4.0-micro-controller
This package is a flight controller intended to simply stabilize an otherwise unstable vehicle platform
such as a multirotor, small helicopter, or other unique VTOL platform. It may also be used to
supplement passively stable platforms such as airplanes, and can even be used on other types
of robotics projects that don’t fly. This is not a flight computer, which would handle advanced
autonomy, computer vision, or path planning. However, dRehmFlight can be modified to accept a
serial input with simulated radio pilot commands, which could be generated by a separate onboard
companion computer. This is outside the scope of this project. dRehmFlight is meant to accept
radio commands from a ground-based pilot, interpret them as vehicle state setpoints (desired vehicle
angle or rotation rate), and have the vehicle perform those commands.
The code is designed to run on a Teensy 4.0 microcontroller with the MPU6050 IMU which
together cost less than $30. The Teensy 4.0 is an extremely powerful microcontroller with 40 digital
pins, 31 of which are PWM-enabled, and all of which are interrupt capable. It’s ARM Cortex-M7
microprocessor features a 600 MHz clock speed enabling a flight control loop rate of 2 kHz with
plenty of overhead for custom code additions. The code is compiled and uploaded to the board
using the simple-to-use Arduino IDE and the Teensyduino add-on

# Hardware Requirements
The physical flight controller which runs the dRehmFlight code consists of a Teensy 4.0 microcontroller and MPU6050 6 degree of freedom IMU. Currently, only the Teensy 4.0 board has been
tested, though the newer 4.1 board with more pinouts should work identically without modification
to pinouts.

# Software Requirements
The flight control code is written in the Arduino/C language and is uploaded to the board using
the Arduino IDE. To connect to the Teensy, you must also download and install the Teensyduino
Arduino add-on.

# The Code
The main code is contained entirely within the dRehmFlight Arduino sketch (with the exception
of radio communication functions which are included separately in the radioComm file in the same
folder). Figure 18 shows the general sequence of contents that can be found in the main Arduino
sketch.
The user-specified defines section is where receiver type, IMU type, and IMU sensitivity values
are selected. The user-specified variable declaration section contains numerical variables such as
failsafe values or controller gains which can be readily changed or tuned. Pins are assigned and
declared in the pin declaration section (all sections are marked by comments within the code as
well). Most variables used throughout the code are global variables. This is generally bad practice
for coding, but allows for access to nearly every variable and parameter wherever we want within
the code. All new variables created can be declared here. Next, the void setup section contains
the setup code and functions which are executed one time on startup.  expands on the
processes occurring within the void setup. The void loop section contains the flight control code
and functions which is run continuously after initial setup. Figure 19 shows the general architecture
for a flight control loop and  expands on the specific processes occurring within the void
loop to achieve this functionality. Finally, all of the critical functions which are called within the
void setup and void loop are contained at the end of the Arduino sketch. A specific operation
is performed within each function, and they have been segmented in such a way that these key
operations are easily understood and followed.
