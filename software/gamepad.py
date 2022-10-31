#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
example class to use the controller with asyncio
python>=3.6 is necessary
script is tested on a raspberry pi 3
"""
import asyncio
from evdev import InputDevice, ff, ecodes

class gamepad():
    def __init__(self, file = '/dev/input/event12'):
        #self.event_value = 0
        self.power_on = True
        self.device_file = InputDevice(file)
        #print(str(self.device_file))
        self.joystick_left_y = 0 # values are mapped to [-1 ... 1]
        self.joystick_left_x = 0 # values are mapped to [-1 ... 1]
        self.joystick_right_x = 0 # values are mapped to [-1 ... 1]
        self.trigger_right = 0 # values are mapped to [0 ... 1]
        self.trigger_left = 0 # values are mapped to [0 ... 1]
        self.button_x = False
        self.button_y = False
        self.button_b = False
        self.rumble_effect = 0
        self.effect1_id = 0 # light rumble, played continuously
        self.effect2_id = 0 # strong rumble, played once
        #self.load_effects()


    def read_gamepad_input(self): # asyncronus read-out of events
        max_abs_joystick_left_x = 0xFFFF/2
        uncertainty_joystick_left_x = 2500
        max_abs_joystick_left_y = 0xFFFF/2
        uncertainty_joystick_left_y = 2500
        max_abs_joystick_right_x = 0xFFFF/2
        uncertainty_joystick_right_x = 2000
        max_trigger = 1023

        while True:
        #for event in self.device_file.read_loop():
            #print(str(event))
            event = self.device_file.read_one()
            if event == None:
                break

            if not(self.power_on): #stop reading device when power_on = false
                break
            if event.type == 3: # type is analog trigger or joystick
                if event.code == 1: # left joystick y-axis
                    if -event.value > uncertainty_joystick_left_y:
                        self.joystick_left_y = (-event.value - uncertainty_joystick_left_y) / (max_abs_joystick_left_y - uncertainty_joystick_left_y + 1)
                    elif -event.value < -uncertainty_joystick_left_y:
                        self.joystick_left_y = (-event.value + uncertainty_joystick_left_y) / (max_abs_joystick_left_y - uncertainty_joystick_left_y + 1)
                    else:
                        self.joystick_left_y = 0
                elif event.code == 0: # left joystick x-axis
                    if event.value > uncertainty_joystick_left_x:
                        self.joystick_left_x = (event.value - uncertainty_joystick_left_x) / (max_abs_joystick_left_x - uncertainty_joystick_left_x + 1)
                    elif event.value < -uncertainty_joystick_left_x:
                        self.joystick_left_x = (event.value + uncertainty_joystick_left_x) / (max_abs_joystick_left_x - uncertainty_joystick_left_x + 1)
                    else:
                        self.joystick_left_x = 0
                elif event.code == 3: # right joystick x-axis
                    if event.value > uncertainty_joystick_right_x:
                        self.joystick_right_x = (event.value - uncertainty_joystick_right_x) / (max_abs_joystick_right_x - uncertainty_joystick_right_x + 1)
                    elif event.value < -uncertainty_joystick_right_x:
                        self.joystick_right_x = (event.value + uncertainty_joystick_right_x) / (max_abs_joystick_right_x - uncertainty_joystick_right_x + 1)
                    else:
                        self.joystick_right_x = 0
                elif event.code == 5: # right trigger
                    self.trigger_right = event.value / max_trigger
                elif event.code == 2: # left trigger
                    self.trigger_left = event.value / max_trigger
            if (event.type == 1): # type is button
                if event.code == 307: # button "X" pressed ?
                    self.button_x = True
                if event.code == 308: # button "Y" pressed ?
                    self.button_y = True
                if event.code == 305: # button "B" pressed ?
                    self.button_b = True



