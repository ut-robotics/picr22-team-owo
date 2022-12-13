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
        self.wheel_radius = 0.38 #meters
        self.wheel_distance_from_center = 0.13 #meters
        self.encoder_counts_per_wheel_revolution = self.gearbox_ratio * self.encoder_edges_per_motor_revolution
        self.pid_control_frequency = 50 #Hz
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
        self.thrower_min = 3277
        self.thrower_max = 6554
        self.succ_servo_zero = 4875
        self.succ_servo_in = 6554  
        self.succ_servo_out = 3277
        self.angle_servo_low = 6150 
        self.angle_servo_high = 4725

        self.throw_radius_min = 50
        self.throw_radius_max = 4000
        self.ball_in_robot = False
        self.throwing_angle_data = [{"angle": self.angle_servo_high, "min_r": 0, "max_r": self.throw_radius_max, "slope": 0.207, "constant": 3562},
                                    {"angle": self.angle_servo_low, "min_r": self.throw_radius_min, "max_r": 0, "slope": 0.207, "constant": 3562}]
        # 0.362 3307
        self.active_slope = self.throwing_angle_data[0]["slope"] # Default to long range at the start (start from far corner)
        self.active_constant = self.throwing_angle_data[0]["constant"]
        self.active_angle = self.throwing_angle_data[0]["angle"]
        self.succ_servo_active_speed = self.succ_servo_zero
        self.driving_forward = True

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
        self.stop_all()

    def close(self):
        self.stop_all()
        self.ser.close()
    # End of pyserial stuff

    # Calculates thrower speed from given distance
    def calculate_throw_strength(self, distance):
        if distance == 0:
            return 0
        else:
            # int(distance*0.277 + 413)
            # return int(distance*0.275 + 411)
            # 48 - 2047
            #print(int((distance*self.active_slope + self.active_constant - 48)/(2047-48) * (6554 - 3277) + 3277))
            #print (int((distance*self.active_slope + self.active_constant)))
            #return int((distance*self.active_slope + self.active_constant))
            if self.driving_forward:
                return int(distance * 0.195 + 3589)
            else:
                return int(distance * 0.207 + 3625)

    # Big math, returns speed of a wheel in mainboard units
    def calculate_wheel_speed(self, motor_num, robot_speed, robot_angle, speed_rot):
        wheel_linear_velocity = robot_speed * math.cos(robot_angle - self.wheel_angles[motor_num - 1]) + self.wheel_distance_from_center * speed_rot
        #print("linear", wheel_linear_velocity, robot_speed * math.cos(robot_angle - self.wheel_angles[motor_num - 1]), self.wheel_distance_from_center * speed_rot)
        wheel_angular_speed_mainboard_units = wheel_linear_velocity * self.wheel_speed_to_mainboard_units
        #print("mainboard units", wheel_angular_speed_mainboard_units)

        # if wheel_angular_speed_mainboard_units > 0.2 and wheel_angular_speed_mainboard_units < 1:
        #     wheel_angular_speed_mainboard_units = 1
        # if wheel_angular_speed_mainboard_units < -0.2 and wheel_angular_speed_mainboard_units > -1:
        #     wheel_angular_speed_mainboard_units = -1

        return wheel_angular_speed_mainboard_units

    def stop_all(self):
        self.send_data(0, 0, 0, self.thrower_min, self.succ_servo_zero, self.angle_servo_low)

    # x - sideways, positive to the right
    # y - forward, positive to the front
    # r - rotation, positive anticlockwise
    # While moving it also rotates the thrower with speed calculated from the distance
    def move(self, speed_x, speed_y, speed_r, thrower_distance=0):
        robot_angle = math.atan2(speed_y, speed_x)
        robot_speed = math.sqrt(math.pow(speed_x, 2) + math.pow(speed_y, 2))
        #print("Angle:", robot_angle)
        #print("Speed:", robot_speed)
        if (abs(robot_speed) > self.max_speed) or (abs(speed_r) > self.max_speed * 4):
            self.logger.LOGE("Speed too large, speeds: " +str(speed_x) + " / " + str(speed_y) + " / " + str(speed_r))
            return

        # M1 front left, M2 front right, M3 back
        motor_speeds = []
        for i in range(3):
            motor_speeds.append(int(self.calculate_wheel_speed(i+1, robot_speed, robot_angle, speed_r)))
        self.send_data(motor_speeds[0], motor_speeds[1], motor_speeds[2], self.calculate_throw_strength(thrower_distance), self.succ_servo_active_speed, self.active_angle)
        #print(self.receive_data())

        actual_speed1, actual_speed2, actual_speed3, enc1, enc2, enc3, error, integral, ball_detected = self.receive_data()
        print("motors:", num_format(actual_speed1), num_format(actual_speed2), num_format(actual_speed3), num_format(enc1), num_format(enc2), num_format(enc3), num_format(error[0]), num_format(error[1]), num_format(error[2]), num_format(integral[0]), num_format(integral[1]), num_format(integral[2]))
        self.ball_in_robot = ball_detected


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
        speed_backwheel = int(-sigmoid_controller(cur_object_x, self.middle_x, x_scale=50, y_scale=4)) # x and y scale need testing
        
        front_motor_speeds = []
        for i in range(2):
            front_motor_speeds.append(int(self.calculate_wheel_speed(i+1, speed_y, math.pi/2, 0)))

        self.send_data(front_motor_speeds[0], front_motor_speeds[1], speed_backwheel, 0, self.succ_servo_active_speed, self.active_angle)

    def eating_servo(self, eating_servo_state): # three states: inactive, eating, sending ball to thrower
        if eating_servo_state == Eating_servo_state.OFF:
            self.succ_servo_active_speed = self.succ_servo_zero
        elif eating_servo_state == Eating_servo_state.EAT: # EAT and THROW speeds need to be tested
            self.succ_servo_active_speed = self.succ_servo_in
        elif eating_servo_state == Eating_servo_state.THROW:
            self.succ_servo_active_speed = self.succ_servo_in
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
        self.send_data(0, 0, 0, self.calculate_throw_strength(thrower_distance), self.succ_servo_zero, self.angle_servo_low)
        #print(self.receive_data())

    # Throw ball with given strength (in "raw motor" units)
    # Discrete call not continuous, use within a loop
    # Value range 48 - 2047
    def throw_raw(self, strength, succ_speed, angle):
        if strength < 3277 or strength > 6554:
            self.logger.LOGE("Invalid throw strength")
        if succ_speed < 3277 or succ_speed > 6554:
            self.logger.LOGE("Invalid succ thrower speed")
        if angle < 4700 or angle > 6200:
            self.logger.LOGE("Invalid throw angle")
        self.send_data(0, 0, 0, strength, succ_speed, angle)

    # speed 1,2,3: 2-80 kiirus (samuti ka negatiivsel poolel)
    # thrower: 3277-6554
    # succ servo: 3277 välja max, 4875 paigal, 6554 sisse max
    # angle servo: 6200 all, 4700 ülal, Check this value before sending, no check for safety within firmware
    def send_data(self, speed1, speed2, speed3, thrower_speed, succ_servo, angle_servo):
        int_const = 0.1
        flat_const = 1
        delimiter = 0xAAAA
        data = struct.pack('<hhhHHHfhH', speed1, speed2, speed3, thrower_speed, succ_servo, angle_servo, int_const, flat_const, delimiter)

        self.ser.write(data)

    def receive_data(self):
        received_data = self.ser.read(size=26)
        error = [0,0,0]
        integral = [0.0,0.0,0.0]

        actual_speed1, actual_speed2, actual_speed3, enc1, enc2, enc3, error[0], error[1], error[2], integral[0], integral[1], integral[2], ball_detected = struct.unpack('<hhhhhhhhhhhhH', received_data)
        return actual_speed1, actual_speed2, actual_speed3, enc1, enc2, enc3, error, integral, ball_detected

    # Rotates all wheels with speed 10, useful for sanity checking
    def test_motors(self):
        while(True):
            robot.send_data(5, 5, 5, self.thrower_min, self.succ_servo_zero, self.angle_servo_high)
            print(robot.receive_data())

    # Simple driving test
    def test_driving(self):
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < 2.0:
                print(1)
                robot.move(0, 2, 0)
                print(1.1)
                print(robot.receive_data())
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < 2.0:
                print(2)
                robot.move(2, 0, 0)
                print(2.2)
                print(robot.receive_data())
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < 2.0:
                print(2)
                robot.move(0, -2, 0)
                print(2.2)
                print(robot.receive_data())
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < 2.0:
                print(2)
                robot.move(-2, 0, 0)
                print(2.2)
                print(robot.receive_data())

if __name__ == "__main__":
    logger = Logging(False, True)
    robot = Mainboard(10, logger)
    robot.start()
    try:
        while(True):
            robot.test_motors()
    except KeyboardInterrupt:
        robot.close()
        print("\nExiting")
    
