# AGV Magnetic Tape Following with NAVIQ MTS160 Sensor

This repository provides a Python code example demonstrating how to control an Automated Guided Vehicle (AGV) using a NAVIQ MTS160 magnetic guide sensor. The code is designed to run on a **Raspberry Pi or any computer (Linux/Windows) equipped with a compatible CAN interface** (e.g., PEAK-System PCAN adapters, Kvaser interfaces) supported by the `python-can` library. It utilizes the CAN bus for communication with the sensor and a generic CAN-based motor controller.

## Overview

The script implements a closed-loop control system for a differential drive AGV to follow a magnetic tape laid on the floor. It leverages both the position error and the angle error provided by the MTS160 sensor relative to the tape.

**Key Features:**

* **Dual Sensor Feedback:** Uses both lateral position (`pos`) and tangential angle (`angle`) measurements from the MTS160.
* **Control Strategy:** Implements a control loop where the measured angle acts as a feedforward term for steering direction, and the position error provides feedback to correct lateral deviation.
* **State Machine:** Includes a basic state machine for handling different operational modes like normal tape following, docking procedures (using Navicodes detected by the sensor), and charging.
* **Hardware Platform:** Runs on Raspberry Pi (example) or other computers with `socketcan` or other `python-can` supported interfaces.
* **CAN Communication:** Interfaces with the MTS160 sensor and a generic motor controller via the CAN bus.
* **MTS160 Feature Utilization:** Demonstrates using left/right sensor data streams, marker coordinates, and Navicodes for control and state management based on an assumed track layout (see "Sensor Data Usage" section).
* **Adaptable:** The motor control output can be easily adapted to suit different CAN-based motor controllers.

## Hardware Requirements

1.  **Computer:** Raspberry Pi (e.g., RPi 3B+, 4, 5) or any computer capable of running Python and interfacing with CAN.
2.  **NAVIQ MTS160 Sensor:** Magnetic guide sensor (naviq.com) configured for CAN communication.
3.  **CAN Interface:** A CAN transceiver/adapter compatible with your computer and the `python-can` library (e.g., PiCAN2 HAT for Raspberry Pi, USB-to-CAN adapter like PCAN-USB, Kvaser Leaf).
4.  **Differential Drive AGV:** A robotic platform with two independently driven wheels.
5.  **Motor Controller:** One or two motor controllers that accept speed commands via CAN bus.
    * The example code assumes the controller listens on CAN ID `0x5` for commands and expects data packed as `struct.pack('<hhB', left_rpm, right_rpm, 0)`.
    * It also optionally listens for telemetry (speed feedback) on CAN ID `0x2`.
6.  **Power Supplies:** Appropriate power for the computer, motors, controllers, and sensor.
7.  **CAN Bus:** Proper cabling with termination resistors (typically 120 Ohm at each end).

## Software Requirements & Setup

1.  **Operating System:** An OS compatible with Python and your CAN interface drivers (e.g., Raspberry Pi OS, Linux, Windows).
2.  **Python:** Python 3.x.
3.  **python-can library:** Install using pip:
    ```bash
    pip install python-can
    # Optional: install drivers/plugins if needed for specific hardware, e.g.
    # pip install kvaser-canlib-driver # For Kvaser interfaces on Windows/Linux
    ```
4.  **Configure CAN Interface:** Set up the CAN interface according to your hardware and OS.
    * **For `socketcan` (Linux/Raspberry Pi):**
        ```bash
        # Example for 250kbps bitrate on interface can0
        sudo ip link set can0 type can bitrate 250000
        sudo ip link set can0 up
        ```
    * **For other interfaces (e.g., PCAN, Kvaser):** Ensure drivers are installed and consult `python-can` documentation for specifying the correct `interface` and `channel` when creating the `can.Bus` instance (modification to the script might be needed).
    * You can test the interface using tools like `candump` (Linux) or vendor-specific tools (PCAN-View, Kvaser CanKing).
5.  **Download Code:** Clone or download the `robot_controller.py` script (or the relevant file from this repository).

## Code Structure & Key Components

* **`RobotController` Class:** The main class orchestrating the AGV's operation.
    * **Constants:** Defines speeds, control gains, robot geometry, state parameters, and CAN IDs.
    * **`__init__`:** Initializes state variables and data structures.
    * **State Machine Methods:**
        * `state_machine()`: Top-level state logic runner.
        * `execute_state_action()`: Performs actions based on the current state (usually calculates motor commands).
        * `check_state_transition()`: Evaluates conditions for changing states (based on navicodes, timers, sensor data).
    * **CAN Parsing Methods (`parse_sensor_data`, etc.):** Decode incoming CAN messages from the sensor into structured data (`SensorData`, `MarkerData`, `NavicodeData`).
    * **Control Calculation (`calculate_motor_commands`):** Implements the core control logic using sensor data to compute motor RPMs.
    * **Main Loop (`run`, `process_message`):** Connects to the CAN bus, receives messages, routes them for parsing, triggers control logic, and sends motor commands.
* **Data Classes (`SensorData`, `MarkerData`, etc.):** Simple structures (`dataclasses`) to hold parsed data from different CAN messages.
* **`State` Enum:** Defines the possible operational states of the AGV (e.g., `NORMAL`, `DOCKING`, `CHARGING`).

## Sensor Data Usage (MTS160 Features)

This example utilizes several data outputs from the NAVIQ MTS160 sensor:

1.  **Left/Right Position & Angle Data (from CAN ID `0x181`):**
    * The sensor provides independent measurements for the left and right sides of its detection range (`pos_left`/`angle_left`, `pos_right`/`angle_right`).
    * The `RobotController` script **switches which dataset it uses based on the current state**.

    * **Assumed Track Layout Context:** This strategy of switching between left and right sensor data is based on an assumed track layout for this specific example: a simple main loop of magnetic tape with a docking/charging spur branching off to the **right**.
        * When in the `NORMAL` state on the main loop, using the `left` sensor data allows the AGV to center itself over the tape.
        * When entering (`ENTER_DOCKING`), aligning within (`DOCKING`), or leaving (`EXIT_DOCKING`) the right-hand spur, switching to the `right` sensor data enables the AGV to correctly follow the path onto or off the branch. This might involve following the right edge of the main tape temporarily or centering on the tape within the spur itself, depending on the exact tape configuration relative to the sensor's field of view.

2.  **Marker Coordinates (from CAN ID `0x281`):**
    * The sensor can detect simple magnetic markers placed near the tape and report their coordinates relative to the sensor (`left_marker_x/y`, `right_marker_x/y`).
    * In this script, the `right_marker_y` coordinate is used during the `DOCKING` state to **verify fine alignment** before transitioning to the `CHARGING` state. The AGV proceeds to charge only when the right marker is detected (`right_marker_x != 0`) and its lateral position (`right_marker_y`) is very close to zero.
3.  **Navicodes (from CAN ID `0x381`):**
    * Navicodes are special coded magnetic markers that allow the sensor to report a unique ID when detected.
    * This script uses specific Navicode values to **signal key locations along the path and trigger state transitions**:
        * `Navicode 0`: Typically indicates the main tape segment. Used to transition *back* to the `NORMAL` state after exiting the dock, and also counted in the `NORMAL` state to determine when enough loops are completed to initiate docking.
        * `Navicode 3`: Indicates the final approach point for docking. Used to transition from `ENTER_DOCKING` to the fine-alignment `DOCKING` state.
    * Other Navicode values could be programmed and used for different locations or actions (e.g., speed changes, intersection decisions).

## Control Strategy & Kinematics

The goal of the control system is to maintain the sensor's position directly over the magnetic tape (lateral error $pos \approx 0$) and align the sensor direction of travel with the tape's direction while the AGV moves at a desired forward speed $v$.

**Robot Parameters:**

* $B$: Wheel base distance (m) (`BASE` constant in code).
* $R$: Drive wheel radius (m) (`RADIUS` constant in code).
* $GR$: Motor gear ratio (`GEAR_RATIO` constant in code).
* $L$: Longitudinal distance from the drive wheel axis to the sensor measurement point (m) (`LENGTH` constant in code).

**Control Inputs (from Sensor):**

* $pos$: Lateral position error (m). Selected from `pos_left` or `pos_right`.
* $angle$: Tangential angle error (degrees). Selected from `angle_left` or `angle_right`.

**Control Parameters:**

* $v$: Desired forward speed (m/s). Set by `VELOCITY_...` constants based on state.
* $K_{p\theta}$: Proportional gain for position-to-angle correction (rad/m). Set by `KP_POS_TO_ANGLE`.

**Control Law:**

1.  **Base Steering Angle (Feedforward):** The sensor's angle relative to the tape provides a direct feedforward term for alignment.

$$
\theta_{base} = \text{radians}(angle)
$$

2.  **Position Correction Angle (Feedback):** An additional steering angle is calculated proportionally to the negative position error to steer back towards the centerline.

$$
\Delta\theta_{pos} = -K_{p\theta} \times pos
$$

3.  **Effective Steering Angle:** The base angle and position correction are combined to get the final target steering angle *relative to the robot's current orientation, as seen by the sensor*.

$$
\theta_{eff} = \theta_{base} + \Delta\theta_{pos}
$$

**Kinematic Model:**

The script converts the desired linear speed $v$ and the effective steering angle $\theta_{eff}$ (defined at the sensor location $L$ ahead of the wheel axis) into target angular velocities for the left ($w_l$) and right ($w_r$) wheels (in rad/s).

1.  **Robot Angular Velocity:** The effective steering angle $\theta_{eff}$ at the offset $L$ relates to the required instantaneous angular velocity $\omega$ (rad/s) of the AGV chassis by:

$$
\omega = \frac{v \sin(\theta_{eff})}{L}
$$

This formula links the steering angle needed at the front sensor to the turning rate of the robot's central axis.

2.  **Wheel Angular Velocities (Code Implementation):** The Python code calculates intermediate components directly:
    * Forward component calculation (shared speed adjusted by cosine of effective angle):

$$
v_{fwd\_comp} = \frac{v \cos(\theta_{eff})}{R}
$$

$$
v_{turn\_comp} = \left( \frac{B}{2 \times L \times R} \right) \times v \times \sin(\theta_{eff})
$$

(Note: This turning component is equivalent to $\frac{\omega \times B}{2 \times R}$, derived from the relationship for $\omega$ above). 

Individual wheel speeds (rad/s):

$$
w_l = v_{fwd_comp} + v_{turn_comp}
$$

$$
w_r = v_{fwd_comp} - v_{turn_comp}
$$

**Output Motor Commands:**

The target wheel angular velocities ($w_l, w_r$) are converted to motor Revolutions Per Minute (RPM) using the gear ratio $GR$:

$$
RPM_L = \frac{w_l}{2 \pi} \times 60 \times GR
$$

$$
RPM_R = \frac{w_r}{2 \pi} \times 60 \times GR
$$

These RPM values are then packed into the CAN message sent to the motor controller.

## Configuration & Customization

The script's behavior can be adjusted via constants defined at the beginning of the `RobotController` class:

* **Speeds (`VELOCITY_...`):** Adjust target speeds for different states.
* **Control Gains (`KP_POS_TO_ANGLE`):** **Crucial tuning parameter ($K_{p\theta}$ in math).** Start low and increase gradually until desired responsiveness is achieved without oscillation.
* **Robot Geometry (`BASE`, `LENGTH`, `RADIUS`, `GEAR_RATIO` constants corresponding to $B, L, R, GR$):** **Must** be set accurately to match the physical dimensions and mechanics of your specific AGV. **Remember to rename the `HEIGHT` constant in the code to `LENGTH`** to match the parameter $L$ used here.
* **State Machine (`CHARGING_TIME_S`, `LOOPS_BEFORE_CHARGING`):** Modify docking logic parameters (Navicodes used, timings, etc.).
* **CAN IDs (`MOTOR_COMMAND_ID`, sensor IDs, etc.):** Change these to match the CAN configuration of your sensor and motor controller(s).
* **Motor Command Format:** If your motor controller uses a different CAN ID or data format than assumed (`struct.pack('<hhB', left_rpm, right_rpm, 0)` on ID `0x5`), you **must** modify the packing logic within the `process_message` method where `motor_msg` is created and sent.
* **CAN Interface (in `run` method):** Modify the `can.interface.Bus(...)` call if not using `socketcan` on `can0`. For example: `can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=250000)` for a PCAN-USB adapter.

## Running the Code

1.  Ensure all hardware is connected and powered.
2.  Verify the CAN bus is properly terminated.
3.  Configure and bring up the CAN interface on your computer.
4.  Navigate to the directory containing the script.
5.  Run the script using Python 3:
    ```bash
    python robot_controller.py
    ```
6.  The script will start listening for CAN messages and controlling the motors based on the sensor input and state logic. Monitor the console output for status messages or errors.
7.  To stop the script, press `Ctrl+C`. It should attempt to send a final stop command to the motors.

## Disclaimer

* This code is provided as an example and starting point. It requires careful tuning and testing on your specific hardware setup.
* **Safety is paramount.** This example lacks critical safety features like emergency stops, obstacle detection, or robust error handling required for production AGV systems. Implement appropriate safety measures before operating the AGV unattended or near people.
* Ensure the geometric parameters ($B, L, R$, corresponding to `BASE`, `LENGTH`, `RADIUS` constants in code) accurately reflect your AGV for the kinematic model to be effective.