#!/usr/bin/python3

# General utility and helper functions

# Imports
import time
import numpy as np

# Logging stuff
class Logging():
    def __init__(self):
        self.logging_start_time = time.perf_counter()
        print("Logging started!")
        #print(logging_start_time)
        #print("logs/" + time.ctime(time.time()))
        self.file = open("logs/" + time.ctime(time.time()) + ".txt", "w")
        self.red = '\033[91m'
        self.yellow = '\033[33m'
        self.green = '\u001b[32m'
        self.end = '\033[0m'
        self.previous_state = ""
        self.file_active = False
        return

    def end(self):
        self.LOGI("Logging ended!")
        self.file.close()

    def create_timestamp(self):
        return "[" + '%.3f' % (time.perf_counter() - self.logging_start_time) + "] "

    def write_log(self, message):
        self.file.write(message)

    def LOGE(self, message):
        message = self.create_timestamp() + message
        self.write_log("ERROR   " + message + "\n")
        print(self.red + message + self.end)

    def LOGW(self, message):
        message = self.create_timestamp() + message
        self.write_log("WARNING " + message + "\n")
        print(self.yellow + message + self.end)

    def LOGI(self, message):
        message = self.create_timestamp() + message
        self.write_log("INFO    " + message + "\n")
        print(message)

    def LOGSTATE(self, state, error=False):
        if self.previous_state != state:
            if error:
                print(self.red + self.create_timestamp() + "New state: " + state + self.end)
            else:
                print(self.green + self.create_timestamp() + "New state: " + state + self.end)
            self.previous_state = state
# End of logging stuff


# Sigmoid controller using a logistic function
# current - current value
# target - Value that the we are trying to reach
# x_scale - For scaling in the x-axis, the curve has reached around 90% of its max value around x_scale
# y_scale - For scaling in the y-axis, by default the curve's max value is 1
# Current values smaller than the target output negative values, larger values positive values, to reverse this use negative x_scale or y_scale
def sigmoid_controller(current, target, x_scale = 1, y_scale = 1):
    return (2 / (1 + np.exp(3*(target-current)/x_scale)) - 1) * y_scale


if __name__ == "__main__":
    log = Logging()
    time.sleep(1)
    log.LOGI("Info")
    log.LOGW("Warning")
    log.LOGE("Error")