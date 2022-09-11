#!/usr/bin/python3

from curses import KEY_A1
import math, struct, time, sys
import numpy as np
import serial

class Omni_motion_robot():
    def __init__(self):
        # Some values taken from here:
        # https://ut-robotics.github.io/picr22-home/basketball_robot_guide/software/omni_motion.html
        self.wheel_angles = np.radians([240, 120, 0]) #radians
        self.name = "OWO"
        self.encoder_edges_per_motor_revolution = 64
        self.gearbox_ratio = 18.75 # 19?
        self.wheel_radius = 0.365 #meters
        self.wheel_distance_from_center = 0.15 #meters
        self.encoder_counts_per_wheel_revolution = self.gearbox_ratio * self.encoder_edges_per_motor_revolution
        self.pid_control_frequency = 60 #Hz
        self.pid_control_period = 1 / self.pid_control_frequency #seconds
        self.wheel_speed_to_mainboard_units = self.gearbox_ratio * self.encoder_edges_per_motor_revolution / (2 * math.pi * self.wheel_radius * self.pid_control_frequency)

    # Pyserial stuff
    def start(self):
        self.ser = serial.Serial("/dev/ttyACM0")
        print(self.ser.name)

    # Pyserial stuff
    def close(self):
        self.ser.close()

    # Big math, returns speed of a wheel in mainboard units
    def calculate_wheel_speed(self, motor_num, robot_speed, robot_angle, speed_rot):
        wheel_linear_velocity = robot_speed * math.cos(robot_angle - self.wheel_angles[motor_num - 1]) + self.wheel_distance_from_center * speed_rot
        wheel_angular_speed_mainboard_units = wheel_linear_velocity * self.wheel_speed_to_mainboard_units
        return wheel_angular_speed_mainboard_units

    # x - sideways, positive to the right
    # y - forward, positive to the front
    # r - rotation, positive anticlockwise
    def move(self, speed_x, speed_y, speed_r):
        robot_angle = math.atan2(speed_y, speed_x)
        robot_speed = math.sqrt(math.pow(speed_x, 2) + math.pow(speed_y, 2))
        print("Angle:", robot_angle)
        print("Speed:", robot_speed)

        #M1 front left
        m1 = int(self.calculate_wheel_speed(1, robot_speed, robot_angle, speed_r))
        #M2 front right
        m2 = int(self.calculate_wheel_speed(2, robot_speed, robot_angle, speed_r))
        #M3 back
        m3 = int(self.calculate_wheel_speed(3, robot_speed, robot_angle, speed_r))

        self.send_data(m1, m2, m3, 0)
        print("Cmd:", m1, m2, m3)
        print("Actual:", self.receive_data())


    # Throw ball with given strength
    # Value range 48 - 2047
    def throw(self, strength):
        if 48 <= strength and strength <= 2047:
            self.send_data(0, 0, 0, strength)
            print(self.receive_data())


    def send_data(self, speed1, speed2, speed3, thrower_speed):
        disable_failsafe = 0
        delimiter = 0xAAAA
        data = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, delimiter)
        self.ser.write(data)


    def receive_data(self):
        received_data = self.ser.read(size=8)
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack('<hhhH', received_data)
        return actual_speed1, actual_speed2, actual_speed3, feedback_delimiter


    # Rotates all wheels with speed 10, useful for sanity checking
    def test_motors(self):
        try:
            while(True):
                robot.send_data(10, 10, 10, 0)
                print(robot.receive_data())
        except KeyboardInterrupt:
            print("\nExiting")
            sys.exit()

if __name__ == "__main__":
    robot = Omni_motion_robot()
    robot.start()
    
    try:
        while(True):
            robot.test_motors()
    except KeyboardInterrupt:
        print("\nExiting")
        sys.exit()
    