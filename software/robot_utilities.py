#!/usr/bin/python3

# General utility and helper functions

# Imports
import time
import numpy as np

# Logging stuff
class Logging():
    def __init__(self, config):
        self.logging_start_time = time.perf_counter()
        config_dict = config.get_module_dict("logger")

        self.file_active = config_dict["file_enabled"]
        self.print_active = config_dict["print_enabled"]
        self.red = '\033[91m'
        self.yellow = '\033[33m'
        self.green = '\u001b[32m'
        self.end = '\033[0m'
        self.previous_state = ""

        if self.file_active:
            self.file = open("logs/" + time.ctime(time.time()) + ".txt", "w")
        if not self.file_active:
            print("REMINDER: File logging inactive")
        if not self.print_active:
            print("REMINDER: Print logging inactive")
        return

    def close(self):
        self.LOGI("Logging ended!")
        if self.file_active:
            self.file.close()

    def create_timestamp(self):
        return "[" + '%.3f' % (time.perf_counter() - self.logging_start_time) + "] "

    def write_log(self, message):
        if self.file_active:
            self.file.write(message)

    def LOGE(self, message):
        message = self.create_timestamp() + message
        self.write_log("ERROR   " + message + "\n")
        if self.print_active:
            print(self.red + message + self.end)

    def LOGW(self, message):
        message = self.create_timestamp() + message
        self.write_log("WARNING " + message + "\n")
        if self.print_active:
            print(self.yellow + message + self.end)

    def LOGI(self, message):
        message = self.create_timestamp() + message
        self.write_log("INFO    " + message + "\n")
        if self.print_active:
            print(message)

    def LOGSTATE(self, state, error=False):
        if self.previous_state != state:
            self.previous_state = state
            state = self.create_timestamp() + state
            if self.file_active:
                self.write_log("NEW STATE   " + state + "\n")
            if self.print_active:
                if error:
                    print(self.red + state + self.end)
                else:
                    print(self.green + state + self.end)
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
    log = Logging(True, True)
    time.sleep(1)
    log.LOGI("Info")
    log.LOGW("Warning")
    log.LOGE("Error")