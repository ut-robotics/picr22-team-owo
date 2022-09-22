#!/usr/bin/python3

# General utility and helper functions

# Logging
red = '\033[91m'
yellow = '\033[33m'
end = '\033[0m'

def LOGE(message):
    print(red + message + end)

def LOGW(message):
    print(yellow + message + end)

def LOGI(message):
    print(message)
