#!/usr/bin/python3

# General utility and helper functions

# Imports
import time, math



# Logging stuff
red = '\033[91m'
yellow = '\033[33m'
green = '\u001b[32m'
end = '\033[0m'
previous_state = ""
file_active = False
logging_start_time = 0

def init_logging():
    global file_active, logging_start_time

    logging_start_time = time.perf_counter()
    #print(logging_start_time)
    #print("logs/" + time.ctime(time.time()))

def create_timestamp():
    return "[" + str(round(time.perf_counter() - logging_start_time, 3)) + "] "

def write_log(message):
    return

def LOGE(message):
    print(red + create_timestamp() + message + end)

def LOGW(message):
    print(yellow + create_timestamp() + message + end)

def LOGI(message):
    print(create_timestamp() + message)

def LOGSTATE(state, error=False):
    global previous_state
    if previous_state != state:
        if error:
            print(red + create_timestamp() + "New state: " + state + end)
        else:
            print(green + create_timestamp() + "New state: " + state + end)
        previous_state = state
# End of logging stuff


# Sigmoid controller using a logistic function
# current - current value
# target - Value that the we are trying to reach
# x_scale - For scaling in the x-axis, the curve has reached around 90% of its max value around x_scale
# y_scale - For scaling in the y-axis, by default the curve's max value is 1
# Current values smaller than the target output negative values, larger values positive values, to reverse this use negative x_scale or y_scale
def sigmoid_controller(current, target, x_scale = 1, y_scale = 1):
    return (2 / (1 + math.exp(3*(target-current)/x_scale)) - 1) * y_scale