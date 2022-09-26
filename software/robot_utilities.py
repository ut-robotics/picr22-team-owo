#!/usr/bin/python3

# General utility and helper functions

# Logging
red = '\033[91m'
yellow = '\033[33m'
green = '\u001b[32m'
end = '\033[0m'
previous_state = ""

def LOGE(message):
    print(red + message + end)

def LOGW(message):
    print(yellow + message + end)

def LOGI(message):
    print(message)

def LOGSTATE(state, error=False):
    global previous_state

    if previous_state != state:
        if error:
            print(red + "New state: " + state + end)
        else:
            print(green + "New state: " + state + end)
        previous_state = state
    