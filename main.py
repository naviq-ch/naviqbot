import math
import struct
import time
import datetime
from dataclasses import dataclass
from enum import Enum
from typing import Tuple

import can

class State(Enum):
    """
    Enumeration of the robot's operational states.
    """
    NORMAL = 0
    ENTER_DOCKING = 1
    DOCKING = 2
    CHARGING = 3
    EXIT_DOCKING = 4

@dataclass
class SensorData:
    """
    Data class for storing sensor readings from CAN ID 0x181.
    """
    pos_left: float = 0.0    # Left sensor position error (meters)
    pos_right: float = 0.0   # Right sensor position error (meters)
    angle_left: int = 0      # Left sensor angle error (degrees)
    angle_right: int = 0     # Right sensor angle error (degrees)
    tape_detect: int = 0     # Tape detection status bits
    left_marker: int = 0     # Left marker detection status
    right_marker: int = 0    # Right marker detection status
    counter: int = 0         # Message counter or timestamp ms

@dataclass
class MarkerData:
    """
    Data class for storing marker data from CAN ID 0x281.
    """
    left_marker_x: float = 0.0  # Left marker X position (scaled value, e.g., meters)
    left_marker_y: float = 0.0  # Left marker Y position (scaled value, e.g., meters)
    right_marker_x: float = 0.0 # Right marker X position (scaled value, e.g., meters)
    right_marker_y: float = 0.0 # Right marker Y position (scaled value, e.g., meters)
    # counter: int = 0          # Counter field not used in original parsing for this ID

@dataclass
class NavicodeData:
    """
    Data class for storing navigation code data from CAN ID 0x381.
    """
    navicode: int = 0           # Detected navigation code
    counter: int = 0            # Message counter for detecting new codes

@dataclass
class MotorTelemetry:
    """
    Data class for storing motor telemetry data received from CAN ID 0x2.
    """
    motor1_speed: int = 0       # Motor 1 speed feedback (e.g., RPM)
    motor2_speed: int = 0       # Motor 2 speed feedback (e.g., RPM)

class RobotController:
    """
    Controls a differential drive robot based on sensor data received via CAN bus.
    Implements a state machine for navigation, docking, and charging.
    Calculates motor commands using position error to adjust steering angle.
    """

    # --- Constants ---
    # Speeds
    VELOCITY_NORMAL = 0.5           # Normal tape following speed (m/s)
    VELOCITY_DOCKING_EXIT = 0.25    # Speed when exiting the docking station (m/s)
    VELOCITY_DOCKING_ENTER = 0.1    # Base speed when entering docking station (m/s)

    # Control Gains
    KP_POS_TO_ANGLE = 1.5           # Proportional gain: position error (m) to steering angle (rad). Needs tuning.

    # Robot Geometry
    BASE = 0.45                     # Wheel base distance (meters)
    LENGTH = 0.42                   # Longitudinal distance from wheel axis to sensor (m)
    RADIUS = 0.0625                 # Wheel radius (meters)
    GEAR_RATIO = 12.2               # Motor gear ratio

    # State Machine Parameters
    CHARGING_TIME_S = 90            # Duration to stay in CHARGING state (seconds)
    LOOPS_BEFORE_CHARGING = 2       # Number of times navicode 0 must be seen in NORMAL state before docking

    # CAN Message Masks/IDs
    TAPE_DETECT_MASK = 0x03         # Bitmask for tape detection status
    # LEFT_MARKER_MASK = 0x08       # Bit position for left marker (derived from parsing logic)
    # RIGHT_MARKER_MASK = 0x10      # Bit position for right marker (derived from parsing logic)
    MOTOR_COMMAND_ID = 0x5          # CAN ID for sending motor commands
    SENSOR_DATA_ID = 0x181          # CAN ID for receiving sensor data
    MARKER_DATA_ID = 0x281          # CAN ID for receiving marker data
    NAVICODE_DATA_ID = 0x381        # CAN ID for receiving navicode data
    MOTOR_TELEMETRY_ID = 0x2        # CAN ID for receiving motor telemetry

    # --- Methods ---
    def __init__(self):
        """
        Initializes the RobotController state and data structures.
        """
        self.current_state: State = State.EXIT_DOCKING
        self.last_navicode_counter: int = 0
        self.charging_start_time: datetime.datetime | None = None
        self.sensor_data: SensorData = SensorData()
        self.marker_data: MarkerData = MarkerData()
        self.navicode_data: NavicodeData = NavicodeData()
        self.motor_telemetry: MotorTelemetry = MotorTelemetry()
        self.loop_counter: int = 0 # Counts navicode 0 detections in NORMAL state
        self.should_stop: bool = False

    @staticmethod
    def parse_sensor_data(message: can.Message) -> SensorData | None:
        """
        Parses sensor data (pos, angle, status) from a CAN message (ID 0x181).
        Returns SensorData object or None if malformed.
        """
        expected_length = struct.calcsize('<bbbbB') # 2x signed char, 2x signed char, 1x unsigned char
        if len(message.data) != expected_length:
            return None
        # '<' little-endian; 'b' signed char; 'B' unsigned char
        data = struct.unpack('<bbbbB', message.data)
        status_byte = data[4]
        return SensorData(
            pos_left=data[0] * 0.001,      # Scale factor applied to raw position
            pos_right=data[1] * 0.001,     # Scale factor applied to raw position
            angle_left=data[2],            # Angle in degrees
            angle_right=data[3],           # Angle in degrees
            tape_detect=(status_byte >> 1) & RobotController.TAPE_DETECT_MASK, # Extract tape status bits
            left_marker=(status_byte >> 3) & 0x01, # Extract left marker bit
            right_marker=(status_byte >> 4) & 0x01,# Extract right marker bit
            counter=int(message.timestamp * 1000) # Use timestamp as a counter/ID
        )

    @staticmethod
    def parse_marker_data(message: can.Message) -> MarkerData | None:
        """
        Parses marker coordinate data from a CAN message (ID 0x281).
        Returns MarkerData object or None if malformed.
        """
        expected_length = struct.calcsize('<HHHH') # 4x unsigned short
        if len(message.data) != expected_length:
            return None
        # '<' little-endian; 'H' unsigned short
        data = struct.unpack('<HHHH', message.data)
        return MarkerData(
            left_marker_x=data[0] * 0.1,    # Scale factor applied to raw X coordinate
            left_marker_y=data[1] * 0.1,    # Scale factor applied to raw Y coordinate
            right_marker_x=data[2] * 0.1,   # Scale factor applied to raw X coordinate
            right_marker_y=data[3] * 0.1    # Scale factor applied to raw Y coordinate
        )

    @staticmethod
    def parse_navicode_data(message: can.Message) -> NavicodeData | None:
        """
        Parses navigation code data from a CAN message (ID 0x381).
        Returns NavicodeData object or None if malformed.
        """
        expected_length = struct.calcsize('<HB') # 1x unsigned short, 1x unsigned char
        if len(message.data) != expected_length:
            return None
        # '<' little-endian; 'H' unsigned short; 'B' unsigned char
        data = struct.unpack('<HB', message.data)
        return NavicodeData(
            navicode=data[0],
            counter=data[1]                 # Use provided counter byte
        )

    @staticmethod
    def parse_motor_telemetry(message: can.Message) -> MotorTelemetry | None:
        """
        Parses motor telemetry feedback from a CAN message (ID 0x2).
        Returns MotorTelemetry object or None if malformed.
        """
        # Assuming format based on previous code: 4x signed short?
        expected_length = struct.calcsize('<hhhh') # 4x signed short
        if len(message.data) != expected_length:
            return None
        # '<' little-endian; 'h' signed short
        data = struct.unpack('<hhhh', message.data)
        # Assuming the first two values are the relevant motor speeds
        return MotorTelemetry(motor1_speed=data[0], motor2_speed=data[1])


    def calculate_motor_commands(self, pos: float, angle: int, speed: float) -> Tuple[float, float]:
        """
        Calculates motor RPM commands using differential drive kinematics.
        Uses position error (`pos`) to create a steering angle adjustment.

        Args:
            pos (float): Position error (meters). Positive means deviation
                         in one direction (e.g., right), negative the other.
            angle (int): Angle error from sensor (degrees).
            speed (float): Desired forward speed (m/s).

        Returns:
            Tuple[float, float]: Commands for motor1 and motor2 (RPM).
        """
        # 1. Calculate base steering angle from sensor reading (convert to radians)
        theta_base = math.radians(angle)

        # 2. Calculate steering angle adjustment based on position error
        # Steer towards the line: if pos > 0 (e.g., right of line), need positive (left) angle correction.
        angle_adjustment = self.KP_POS_TO_ANGLE * (-pos) # Adjustment in radians

        # 3. Calculate the total effective steering angle
        theta_effective = theta_base + angle_adjustment # Total angle in radians

        # 4. Calculate kinematic terms using the effective angle
        cos_theta = math.cos(theta_effective)
        sin_theta = math.sin(theta_effective)

        # Geometric factor used in the kinematic model, incorporating LENGTH (sensor offset)
        # This term influences the wheel speed difference required for turning.
        kinematic_factor = self.BASE / (2 * self.LENGTH * self.RADIUS)


        # 5. Calculate required wheel angular velocities (rad/s)
        # Forward motion component, shared by both wheels
        forward_component = (speed * cos_theta) / self.RADIUS
        # Turning component, creating difference between wheels
        turning_component = kinematic_factor * speed * sin_theta

        wl = forward_component + turning_component # Left wheel angular velocity
        wr = forward_component - turning_component # Right wheel angular velocity

        # 6. Convert wheel angular velocities (rad/s) to motor RPM
        motor1_cmd = (wl / (2 * math.pi)) * 60 * self.GEAR_RATIO
        motor2_cmd = (wr / (2 * math.pi)) * 60 * self.GEAR_RATIO

        return motor1_cmd, motor2_cmd

    def state_machine(self) -> Tuple[float, float]:
        """
        Runs the state machine logic: determines actions based on the current
        state, checks for transitions, and returns motor commands.
        """
        new_navicode = self.check_new_navicode()
        motor1_cmd, motor2_cmd = self.execute_state_action()
        self.check_state_transition(new_navicode)
        return motor1_cmd, motor2_cmd

    def check_new_navicode(self) -> bool:
        """
        Checks if a new navicode message has been received since the last check.
        Updates the last known counter.
        """
        is_new = (self.last_navicode_counter != self.navicode_data.counter)
        if is_new:
            self.last_navicode_counter = self.navicode_data.counter
        return is_new

    def execute_state_action(self) -> Tuple[float, float]:
        """
        Determines and executes the primary action for the current state,
        typically calculating motor commands based on sensor inputs.
        """
        # --- Safety Check: Tape Detection ---
        # Requires tape detected (value > 0) for all moving states.
        # Assumes tape_detect uses multiple bits, 0 means no tape.
        moving_states = [State.NORMAL, State.ENTER_DOCKING, State.DOCKING, State.EXIT_DOCKING]
        if self.current_state in moving_states and self.sensor_data.tape_detect == 0:
            print(f"ERROR: Tape lost in state: {self.current_state.name}")
            self.should_stop = True
            return 0.0, 0.0  # Stop motors

        # --- State Actions ---
        if self.current_state == State.CHARGING:
            # Motors off while charging
            return 0.0, 0.0

        elif self.current_state == State.NORMAL:
            # Follow tape using left sensor data
            return self.calculate_motor_commands(
                self.sensor_data.pos_left, self.sensor_data.angle_left, self.VELOCITY_NORMAL
            )

        elif self.current_state == State.ENTER_DOCKING:
            # Approach dock using right sensor data at normal speed initially
            return self.calculate_motor_commands(
                self.sensor_data.pos_right, self.sensor_data.angle_right, self.VELOCITY_NORMAL
            )

        elif self.current_state == State.DOCKING:
            # Final alignment using right sensor data at slower docking speed
            # Speed adjustment based on marker proximity could be added here if needed.
            speed = self.VELOCITY_DOCKING_ENTER
            return self.calculate_motor_commands(
                self.sensor_data.pos_right, self.sensor_data.angle_right, speed
            )

        elif self.current_state == State.EXIT_DOCKING:
            # Exit dock using right sensor data
            return self.calculate_motor_commands(
                self.sensor_data.pos_right, self.sensor_data.angle_right, self.VELOCITY_DOCKING_EXIT
            )

        # Default case (should not be reached in normal operation)
        return 0.0, 0.0

    def check_state_transition(self, new_navicode: bool):
        """
        Checks conditions for transitioning between states based on sensor data
        (navicodes, marker positions, timers).
        """
        current_state = self.current_state # Cache for readability

        if current_state == State.NORMAL:
            # Condition: See navicode 0 enough times -> Enter Docking
            if new_navicode and self.navicode_data.navicode == 0:
                self.loop_counter += 1
                if self.loop_counter >= self.LOOPS_BEFORE_CHARGING:
                    self.current_state = State.ENTER_DOCKING
                    self.loop_counter = 0 # Reset counter for next docking cycle

        elif current_state == State.ENTER_DOCKING:
            # Condition: See navicode 3 -> Start Final Docking Alignment
            if new_navicode and self.navicode_data.navicode == 3:
                self.current_state = State.DOCKING

        elif current_state == State.DOCKING:
            # Condition: Right marker detected and Y position near zero -> Charge
            # Use a small threshold for Y alignment check.
            alignment_threshold_y = 0.05 # Meters (adjust based on marker_data scaling)
            is_marker_detected = (self.marker_data.right_marker_x != 0)
            is_aligned = (abs(self.marker_data.right_marker_y) < alignment_threshold_y)

            if is_marker_detected and is_aligned:
                self.current_state = State.CHARGING
                self.charging_start_time = datetime.datetime.now()

        elif current_state == State.CHARGING:
            # Condition: Charging time elapsed -> Exit Docking
            if self.charging_start_time and \
               (datetime.datetime.now() - self.charging_start_time).total_seconds() > self.CHARGING_TIME_S:
                self.current_state = State.EXIT_DOCKING
                self.charging_start_time = None # Clear timer

        elif current_state == State.EXIT_DOCKING:
            # Condition: See navicode 0 -> Return to Normal tape following
            if new_navicode and self.navicode_data.navicode == 0:
                self.current_state = State.NORMAL
                # loop_counter is already 0 or will be reset next time navicode 0 is seen in NORMAL

        # Optional: Print state transitions for debugging
        # if current_state != self.current_state:
        #     print(f"State transition: {current_state.name} -> {self.current_state.name}")


    def run(self):
        """
        Main execution loop: connects to CAN, receives messages, processes them,
        and sends motor commands until stop condition is met.
        """
        try:
            # Use 'with' for automatic bus shutdown
            with can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False) as bus:
                print(f"RobotController started. Listening on {bus.channel_info}...")
                while not self.should_stop:
                    # Receive CAN message with timeout
                    message = bus.recv(timeout=1.0) # seconds

                    if message:
                        # Process the received message
                        self.process_message(message, bus)
                    # else:
                        # Optional: Handle timeout (e.g., check system health)
                        # print("CAN receive timeout.")
                        # pass

                    # Check if processing requested a stop
                    if self.should_stop:
                        print("Stop requested, sending zero motor commands...")
                        # Attempt to send a final stop command
                        motor_data = struct.pack('<hhB', 0, 0, 0) # Zero speed for both motors
                        stop_msg = can.Message(arbitration_id=self.MOTOR_COMMAND_ID, data=motor_data, is_extended_id=False)
                        try:
                            bus.send(stop_msg)
                            print("Sent stop command.")
                        except can.CanError as e:
                             print(f"Error sending final stop command: {e}")
                        break # Exit the while loop

        except can.CanError as e:
             # Specific CAN interface errors
             print(f"CAN interface error: {e}")
             self.should_stop = True # Ensure stop if CAN fails
        except Exception as e:
            # Catch unexpected errors during operation
            print(f"An unexpected error occurred in RobotController run loop: {e}")
            import traceback
            traceback.print_exc() # Print full traceback for debugging
            self.should_stop = True # Ensure stop on unexpected errors
        finally:
            # This block executes whether the loop finishes normally or due to an error
            print("RobotController has stopped.")


    def process_message(self, message: can.Message, bus: can.Bus):
        """
        Routes incoming CAN messages to the appropriate parsing function
        and triggers control logic or updates internal data.

        Args:
            message (can.Message): The received CAN message.
            bus (can.Bus): The CAN bus interface (needed for sending commands).
        """
        arbitration_id = message.arbitration_id
        parser = None
        data_target = None

        # --- Map CAN IDs to Parsers and Data Targets ---
        if arbitration_id == self.SENSOR_DATA_ID:
            parser = self.parse_sensor_data
            data_target = 'sensor_data'
        elif arbitration_id == self.MARKER_DATA_ID:
            parser = self.parse_marker_data
            data_target = 'marker_data'
        elif arbitration_id == self.NAVICODE_DATA_ID:
            parser = self.parse_navicode_data
            data_target = 'navicode_data'
        elif arbitration_id == self.MOTOR_TELEMETRY_ID:
            parser = self.parse_motor_telemetry
            data_target = 'motor_telemetry'
        # else:
             # Silently ignore unknown CAN IDs
             # print(f"Ignoring unknown CAN ID: {arbitration_id:#05x}")
             # return

        # --- Parse and Update Data ---
        if parser and data_target:
            parsed_data = parser(message)
            if parsed_data:
                setattr(self, data_target, parsed_data)
                # print(f"Updated {data_target}: {parsed_data}") # Debug print

                # --- Trigger Control Logic ONLY after receiving primary sensor data ---
                if data_target == 'sensor_data' and not self.should_stop:
                    # Run state machine to get latest commands
                    motor1_cmd, motor2_cmd = self.state_machine()

                    # Pack and send motor commands
                    # '<' little-endian; 'h' signed short; 'B' unsigned char (for flags/unused byte)
                    motor_data = struct.pack('<hhB', int(motor1_cmd), int(motor2_cmd), 0)
                    motor_msg = can.Message(arbitration_id=self.MOTOR_COMMAND_ID, data=motor_data, is_extended_id=False)
                    try:
                        bus.send(motor_msg)
                    except can.CanError as e:
                        print(f"Error sending motor command (ID {self.MOTOR_COMMAND_ID:#05x}): {e}")
                        self.should_stop = True # Stop if communication fails

            # else:
                # Parsing failed (malformed message) - ignore silently
                # print(f"Failed to parse message ID {arbitration_id:#05x}")
                # pass


if __name__ == "__main__":
    print("Starting Robot Controller...")
    controller = RobotController()
    controller.run()
    print("Robot Controller Finished.")