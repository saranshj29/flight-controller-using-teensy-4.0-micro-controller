
## 1. Introduction

The **dRehmFlight** project aims to provide a comprehensive toolkit for building and operating VTOL (Vertical Takeoff and Landing) vehicles. It integrates a powerful flight controller with a straightforward coding framework that accommodates users of varying skill levels. The project’s design philosophy prioritizes accessibility, allowing both novices and experienced hobbyists to create their unique VTOL platforms.

## 2. Project Overview

The project centers around a simple yet effective flight controller designed to stabilize different types of VTOL vehicles, from multirotors to complex transitioning models. It emphasizes modularity, allowing users to modify the underlying code and hardware configurations to suit their specific requirements. 

### Goals of the Project
- To facilitate the understanding of flight control systems.
- To provide a robust platform for both educational and research purposes.
- To enable customization and flexibility in design.

## 3. Hardware Requirements

### 3.1 Teensy Microcontroller
The **Teensy 4.0** microcontroller serves as the core processing unit of the dRehmFlight system. Its high processing speed and flexibility make it suitable for real-time applications like flight control. 

#### Key Features:
- **Clock Speed**: 600 MHz ARM Cortex-M7 processor.
- **Digital Pins**: 40 digital I/O pins, of which 31 support PWM.
- **Communication Interfaces**: Includes I2C, SPI, CAN bus, and UART for diverse sensor and actuator connectivity.
- **Memory**: Adequate RAM and flash storage to handle the code and data processing required for flight control.

### 3.2 Inertial Measurement Unit (IMU)
The **MPU6050** IMU is used for measuring the vehicle's orientation and motion. It combines a 3-axis gyroscope and a 3-axis accelerometer, providing critical data for maintaining stability during flight.

#### Specifications:
- **Degrees of Freedom**: 6DOF (3-axis accelerometer, 3-axis gyroscope).
- **Communication**: Utilizes I2C for data transfer to the Teensy microcontroller.
- **Calibration**: The system averages readings to establish baseline measurements for error correction.

### 3.3 Electronic Speed Controllers (ESCs)
ESCs are vital for controlling the speed of the motors in the VTOL vehicle. They receive signals from the flight controller and adjust the motor speeds accordingly.

#### Features:
- **Protocol Support**: The code supports OneShot125 for fast communication and response.
- **Compatibility**: Works with various brushless motors suitable for UAV applications.

### 3.4 Servos and Actuators
Servos are used for control surfaces like elevons or ailerons. They provide the necessary aerodynamic control for maneuvering the aircraft.

#### Considerations:
- **Torque and Speed**: Choose servos based on the vehicle's size and weight requirements.
- **PWM Control**: Standard servos operate on PWM signals for position control.

### 3.5 Power Supply
The system requires a reliable power source to ensure stable operation. The recommended approach is to use an ESC's built-in BEC or an external battery.

#### Voltage Requirements:
- **Teensy**: Operates at 3.3V logic.
- **Servos**: Typically powered by a 5V supply.

## 4. Hardware Setup

### 4.1 Suggested Configuration
The recommended hardware setup minimizes complexity and soldering while ensuring compatibility with the default code. 

#### Assembly Steps:
1. **Solder Pin Headers**: Attach male pin headers to the Teensy for easy access to the I/O pins.
2. **Connect the IMU**: Wire the MPU6050 to the appropriate I2C pins on the Teensy (typically pins 18 and 19).
3. **Connect ESCs and Servos**: Ensure all motor and servo connections are made according to the pinout configuration.

### 4.2 Wiring Diagram
![Wiring Diagram](https://example.com/wiring-diagram)  *(Insert a link or actual image of the wiring diagram here)*

### 4.3 Alternative Configurations
Users may opt for custom setups based on their specific requirements, including using different microcontrollers or sensors. The flexibility of the code allows for modifications to accommodate various hardware.

## 5. Software Requirements

### 5.1 Development Environment
To work on the dRehmFlight project, the following software tools are necessary:

- **Arduino IDE**: The primary platform for writing and uploading code to the Teensy.
- **Teensyduino Add-On**: This extension allows the Arduino IDE to recognize and work with Teensy microcontrollers.

### 5.2 Code Structure
The code is modular, with distinct sections handling different functionalities:

- **Initialization**: Configures hardware and sets up communication.
- **Control Loops**: Implements the main flight control algorithms.
- **Data Processing**: Handles data from the IMU and processes input from the radio receiver.

### 5.3 Important Libraries
The project uses several libraries for communication and data processing:
- **Wire Library**: For I2C communication with the IMU.
- **PID Library**: For implementing PID control algorithms.
- **Servo Library**: For controlling servo motors.

## 6. Software Setup

### 6.1 Downloading the Code
1. **Access GitHub Repository**: The dRehmFlight code can be found at: [GitHub Link](https://github.com/nickrehm/dRehmFlight).
2. **Download and Unzip**: Save the repository to your computer and extract the files.

### 6.2 Setting Up the Arduino IDE
1. **Install Arduino IDE**: Download the latest version from the [Arduino website](https://www.arduino.cc/en/software).
2. **Install Teensyduino**: Follow the instructions on the [PJRC website](https://www.pjrc.com/teensy/td_download.html) to install the Teensyduino add-on.

### 6.3 Uploading the Code
1. **Open the dRehmFlight Sketch**: Launch the Arduino IDE and open the main sketch file.
2. **Select Board and COM Port**: Navigate to `Tools > Board` and choose "Teensy 4.0." Select the correct COM port under `Tools > Port`.
3. **Upload**: Click the upload button in the IDE to compile and upload the code to the Teensy.

## 7. Control Logic

### 7.1 Flight Control Algorithms
The flight controller uses a series of algorithms to ensure stable flight. The primary algorithm is a PID control loop, which regulates the vehicle's orientation and altitude based on feedback from the IMU and user inputs.

### 7.2 PID Control Implementation
The PID controller continuously adjusts the control inputs based on the error between the desired state and the actual state of the vehicle. 

- **Proportional Control (P)**: Reacts to the current error.
- **Integral Control (I)**: Accumulates past errors to eliminate steady-state error.
- **Derivative Control (D)**: Predicts future errors based on the rate of change.

### 7.3 Mixing Control Outputs
The control mixer takes the outputs from the PID controllers and assigns them to the appropriate motors and servos. This allows for customized vehicle behavior based on its design. For example, a quadcopter's motors are assigned in a way that tilting the vehicle will cause the motors on one side to increase power while decreasing power on the opposite side.

## 8. Tuning and Calibration

### 8.1 Initial Calibration of the IMU
To achieve accurate flight control, the IMU must be calibrated. This involves taking multiple readings at startup and calculating offsets for the accelerometer and gyroscope.

#### Calibration Steps:
1. **Gather Raw Data**: Take several readings while the vehicle is stationary.
2. **Calculate Offsets**: Determine average values for each axis to use as calibration offsets.
3. **Apply Offsets**: Modify the code to apply these offsets during data processing.

### 8.2 PID Tuning Procedure
Tuning the PID controller is crucial for achieving responsive and stable flight. 

### 8.2 PID Tuning Procedure (continued)

**Tuning Steps:**
1. **Start with Default Values**: Begin with the recommended PID values found in the documentation or use conservative estimates. Common initial values might be:
   - **Proportional Gain (Kp)**: Start with a low value (e.g., 0.1) to see how the vehicle responds.
   - **Integral Gain (Ki)**: Set to zero initially.
   - **Derivative Gain (Kd)**: Set to zero initially as well.

2. **Increase Proportional Gain**: Gradually increase the Kp value until you observe oscillations in the vehicle's response. The goal is to find the point just before the system becomes unstable. 

3. **Introduce Integral Gain**: After determining a satisfactory Kp, slowly introduce the Ki value. This should help eliminate any steady-state error you might see after the vehicle has settled. Start with a low value (e.g., 0.01) and observe the effects.

4. **Add Derivative Gain**: Finally, introduce Kd to dampen any oscillations. This value is usually kept smaller compared to Kp. Adjusting Kd can help reduce overshoot and improve stability.

5. **Iterate**: After each adjustment, perform flight tests to evaluate performance. Note down the behavior and repeat the tuning process as necessary, adjusting one parameter at a time to observe its effects.

### 8.3 Performance Testing
Once PID gains are tuned, conduct thorough performance testing. This phase ensures that the VTOL responds accurately to user inputs and maintains stability under various conditions. 

#### Testing Scenarios:
- **Hover Test**: Check how well the vehicle maintains a stable position. Ensure it does not drift significantly in any direction.
- **Pitch and Roll Test**: Maneuver the vehicle through gentle tilts to observe how it recovers to a stable position. Ensure it responds smoothly without excessive oscillation.
- **Yaw Control**: Test the yaw control by rotating the vehicle and checking its ability to maintain a heading.
- **Transition Testing**: If the vehicle is a hybrid or transitioning model, test its ability to switch between hover and forward flight modes smoothly.

Document the performance in various scenarios to identify areas requiring further tuning.



## 10. Conclusion

The **dRehmFlight VTOL project** serves as a comprehensive platform for building and operating versatile UAVs. Its integration of hardware and software components offers a flexible and user-friendly approach to aerial vehicle design, making it accessible to enthusiasts and researchers alike.

Through meticulous attention to both the hardware assembly and software coding, users can create customized VTOL solutions that cater to a variety of applications. The project encourages experimentation and innovation, pushing the boundaries of what is possible in the field of UAV technology.

### Future Work
Future enhancements for the dRehmFlight project could include:
- **Autonomous Flight Capabilities**: Implementing advanced algorithms for autonomous navigation and path planning.
- **Enhanced Sensor Integration**: Adding support for more complex sensors such as LiDAR, GPS, or computer vision systems.
- **Community Contributions**: Encouraging users to share their modifications and experiences to foster collaboration and collective learning.

## 11. References
- dRehmFlight GitHub Repository: [GitHub Link](https://github.com/nickrehm/dRehmFlight)
- Teensy Microcontroller Documentation: [PJRC Website](https://www.pjrc.com/teensy/)
- MPU6050 Datasheet: [InvenSense Documentation](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- Arduino IDE Documentation: [Arduino Official Site](https://www.arduino.cc/en/Guide/HomePage)

---

This documentation should provide a comprehensive view of the dRehmFlight VTOL project, covering essential components, setup procedures, control logic, tuning, and user guidance. Let me know if you need any additional information or adjustments!
