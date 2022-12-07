#!/usr/bin/python3

import math, struct, time, sys
import numpy as np
import serial, serial.tools.list_ports
from robot_utilities import *
from enum import Enum

class Eating_servo_state(Enum):
    OFF = 1
    EAT = 2
    THROW = 3

class SerialPortNotFound(Exception):
    def __init__(self):
        super().__init__("Serial port with given hardware id not found")

class Mainboard():
    def __init__(self, max_speed, logger):
        # General movement constants
        # Some values taken from here:
        # https://ut-robotics.github.io/picr22-home/basketball_robot_guide/software/omni_motion.html
        self.wheel_angles = np.radians([240, 120, 0]) #radians
        self.encoder_edges_per_motor_revolution = 64
        self.gearbox_ratio = 18.75 # 19?
        self.wheel_radius = 0.365 #meters
        self.wheel_distance_from_center = 0.15 #meters
        self.encoder_counts_per_wheel_revolution = self.gearbox_ratio * self.encoder_edges_per_motor_revolution
        self.pid_control_frequency = 60 #Hz
        self.pid_control_period = 1 / self.pid_control_frequency #seconds
        self.wheel_speed_to_mainboard_units = self.gearbox_ratio * self.encoder_edges_per_motor_revolution / (2 * math.pi * self.wheel_radius * self.pid_control_frequency)
        self.max_speed = max_speed
        self.previous_x = 0
        self.previous_y = 0
        self.previous_r = 0
        self.acceleration_limit = 1

        # Mainboard communication
        self.mainboard_hwid = "USB VID:PID=0483:5740"
        self.baud_rate = 115200

        # Orbiting constants
        self.buffer_r = 1
        self.buffer_x = 1
        self.middle_x = 424

        self.r_const = 5
        self.y_const = 1
        self.prev_rad = 400

        # New robot throwing logic
        self.eating_servo_speed = 0
        self.ball_in_robot = False
        self.throwing_angle_data = [{"angle": 59, "min_r": 1900, "max_r": 2500, "slope": 0.275, "constant": 420},
                                    {"angle": 63, "min_r": 900, "max_r": 2000, "slope": 0.275, "constant": 420},
                                    {"angle": 67, "min_r": 400, "max_r": 1000, "slope": 0.275, "constant": 420}]
        self.active_slope = self.throwing_angle_data[0]["slope"] # Default to long range at the start (start for far corner)
        self.active_constant = self.throwing_angle_data[0]["constant"]
        self.angle = self.throwing_angle_data[0]["angle"]

        # Logger
        self.logger = logger

    # Pyserial stuff
    def start(self):
        #self.ser = serial.Serial("/dev/ttyACM0")
        #print(self.ser.name)
      
        port_list = serial.tools.list_ports.comports()
        devices = {}
        serial_port = None

        for port, _, hwid in sorted(port_list):
            devices[hwid] = port

        for hwid in devices.keys():
            if self.mainboard_hwid in hwid:
                serial_port = devices[hwid]
                break
        
        if serial_port is None:
            self.logger.LOGE("Cannot connect to mainboard")
            raise SerialPortNotFound 
        self.ser = serial.Serial(serial_port, self.baud_rate)

    def close(self):
        self.ser.close()
    # End of pyserial stuff

    # Calculates thrower speed from given distance
    def calculate_throw_strength(self, distance):
        if distance == 0:
            return 0
        else:
            # int(distance*0.277 + 413)
            # return int(distance*0.275 + 411)
            return int(distance*self.active_slope + self.active_constant)

    # Big math, returns speed of a wheel in mainboard units
    def calculate_wheel_speed(self, motor_num, robot_speed, robot_angle, speed_rot):
        wheel_linear_velocity = robot_speed * math.cos(robot_angle - self.wheel_angles[motor_num - 1]) + self.wheel_distance_from_center * speed_rot
        wheel_angular_speed_mainboard_units = wheel_linear_velocity * self.wheel_speed_to_mainboard_units
        return wheel_angular_speed_mainboard_units

    # x - sideways, positive to the right
    # y - forward, positive to the front
    # r - rotation, positive anticlockwise
    # While moving it also rotates the thrower with speed calculated from the distance
    def move(self, speed_x, speed_y, speed_r, thrower_distance=0):
        robot_angle = math.atan2(speed_y, speed_x)
        robot_speed = math.sqrt(math.pow(speed_x, 2) + math.pow(speed_y, 2))
        #print("Angle:", robot_angle)
        #print("Speed:", robot_speed)
        if (abs(robot_speed) > self.max_speed) or (abs(speed_r) > self.max_speed * 2):
            self.logger.LOGE("Speed to large, speeds: " +str(speed_x) + " / " + str(speed_y) + " / " + str(speed_r))
            return

        # M1 front left, M2 front right, M3 back
        motor_speeds = []
        for i in range(3):
            motor_speeds.append(int(self.calculate_wheel_speed(i+1, robot_speed, robot_angle, speed_r)))

        self.send_data(motor_speeds[0], motor_speeds[1], motor_speeds[2], self.calculate_throw_strength(thrower_distance))
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = self.receive_data()
        #print("Actual:", self.receive_data())

        # +++ NEW ROBOT STUFF, uncomment this +++
        # act_speed1, act_speed2, act_speed3, ball_in, delim = self.receive_data()

    # Orbit around something with constant linear (sideways) speed, cur values allow for adjustments in speed, to make it more precise
    # speed - positive value starts orbiting anticlockwise (robot moves right)
    # radius, cur_radius in millimeters
    # cur_object_x - x coordinate of the center of the orbital trajectory
    def orbit(self, radius, speed_x, cur_radius, cur_object_x):
        speed_y = 0
        speed_r = 1000 * speed_x / radius

        # Correct radius check
        if cur_radius > 600:
            self.logger.LOGE("Invalid radius, radius: " + str(cur_radius))
        #    return
        # Radius adjustment
        if cur_radius > (radius + self.buffer_x) or cur_radius < (radius - self.buffer_x):
            speed_y -= (radius - cur_radius) / 100 * self.y_const
        # Centering object, rotational speed adjustment
        if cur_object_x > (self.middle_x + self.buffer_x) or cur_object_x < (self.middle_x - self.buffer_x):
            speed_r += (self.middle_x - cur_object_x) / 100 * self.r_const
        #self.logger.LOGI("orbit speeds x: " + str(speed_x) + " y: " + str(speed_y) + " r: " + str(speed_r) + " cur_object_x: " + str(cur_object_x))

        #print("x:", speed_x, "y:", speed_y, "r:", speed_r, "cur_rad:", cur_radius, "cur_x:", cur_object_x)
        self.move(speed_x, speed_y, speed_r)
        self.prev_rad = cur_radius

    # Moves forward with constant speed with the first two wheel and makes direction adjustments with the back wheel
    def move_backwheel_adjust(self, speed_y, cur_object_x):
        speed_backwheel = -sigmoid_controller(cur_object_x, self.middle_x, x_scale=50, y_scale=4) # x and y scale need testing
        
        front_motor_speeds = []
        for i in range(2):
            front_motor_speeds.append(int(self.calculate_wheel_speed(i+1, speed_y, math.pi/2, 0)))

        self.send_data(front_motor_speeds[0], front_motor_speeds[1], speed_backwheel, 0)
        
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = self.receive_data()
        # +++ NEW ROBOT STUFF, uncomment this +++
        # act_speed1, act_speed2, act_speed3, ball_in, delim = self.receive_data()

    def eating_servo(self, eating_servo_state): # three states: inactive, eating, sending ball to thrower
        if eating_servo_state == Eating_servo_state.OFF:
            self.eating_servo_speed = 0
        elif eating_servo_state == Eating_servo_state.EAT: # EAT and THROW speeds need to be tested
            self.eating_servo_speed = 1
        elif eating_servo_state == Eating_servo_state.THROW:
            self.eating_servo_speed = 2
        else:
            logger.LOGE("Invalid eating_servo_state")

    def choose_thrower_angle(self, basket_distance):
        for i in range(len(self.throwing_angle_data)):
            if basket_distance >= self.throwing_angle_data[i]["min_r"] and basket_distance <= self.throwing_angle_data[i]["max_r"]:
                self.active_constant = self.throwing_angle_data[i]["constant"]
                self.active_slope = self.throwing_angle_data[i]["slope"]
                self.active_angle = self.throwing_angle_data[i]["angle"]
                return

    # Throw ball to a basket at given distance
    # Discrete call not continuous, use within a loop
    def throw(self, thrower_distance):
        self.send_data(0, 0, 0, self.calculate_throw_strength(thrower_distance))
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = self.receive_data()
        #print(self.receive_data())

    # Throw ball with given strength (in "raw motor" units)
    # Discrete call not continuous, use within a loop
    # Value range 48 - 2047
    def throw_raw(self, strength):
        if 48 <= strength and strength <= 2047:
            self.send_data(0, 0, 0, strength)
            actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = self.receive_data()
            #print(self.receive_data())

    def send_data(self, speed1, speed2, speed3, thrower_speed):
        disable_failsafe = 0
        delimiter = 0xAAAA
        data = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, delimiter) # TODO add active angle, eating servo speed for new robot
        self.ser.write(data)

    def receive_data(self):
        received_data = self.ser.read(size=8)
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack('<hhhH', received_data) #TODO add ball-in 
        return actual_speed1, actual_speed2, actual_speed3, feedback_delimiter # TODO add ball-in


    # Rotates all wheels with speed 10, useful for sanity checking
    def test_motors(self):
        try:
            while(True):
                robot.send_data(10, 10, 10, 0)
                print(robot.receive_data())
        except KeyboardInterrupt:
            print("\nExiting")
            sys.exit()

    # Simple driving test
    def test_driving(self):
        try:
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < 2.0:
                robot.move(0, 2, 1)
                print(robot.receive_data())

            start_time = time.perf_counter()
            while time.perf_counter() - start_time < 2.0:
                robot.move(0, -2, -1)
                print(robot.receive_data())
        except KeyboardInterrupt:
            print("\nExiting")
            sys.exit()

if __name__ == "__main__":
    logger = Logging()
    robot = Mainboard(10, logger)
    robot.start()
    try: 
        while(True):
            robot.throw(200)
    except KeyboardInterrupt:
        print("\nExiting")
    
