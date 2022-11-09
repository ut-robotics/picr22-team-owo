#!/usr/bin/python3

# Python
import time, sys, math
from enum import Enum

# Our code
import gamepad
import mainboard
from robot_utilities import *
import image_processor
import camera
import ref_cmd

# External dependencies
import cv2

class TargetBasket(Enum):
    MAGENTA = 1
    BLUE = 2

class State(Enum):
    WAIT = 1
    PAUSED = 2
    START_WAIT = 3
    START_GO = 4
    BALL_SEARCH = 5
    BALL_MOVE = 6
    BALL_ORBIT = 7
    BALL_THROW = 8
    # These 2 are for thrower calibration
    CALIB_INPUT = 9
    CALIB_THROW = 10
    # Control with XBox controller
    MANUAL = 11

if __name__ == "__main__":
    # Housekeeping setup
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    log = Logging(True, True)
    log.LOGI("Starting...")
    debug = False

    # Mainboard stuff setup
    max_speed = 10
    max_speed_inner = 15
    ball_good_range = 300
    robot = mainboard.Mainboard(max_speed_inner, log)
    robot.start()

    # Control logic setup
    debug = False
    state = State.START_WAIT # <====================================================== Initial state here!
    thrower_speed = 0
    calib_first_time = True
    calibration_data = []
    basket_color = TargetBasket.MAGENTA # currently defaults to magenta for testing purposes
    thrower_time_start = 0
    start_go_time_start = 0
    start_go_first_time = True

    # Vision setup
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()

    # Manual control
    xboxcont = None
    manualcontrol = False

    # Constants etc.
    # Housekeeping
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    # Camera constants
    middle_x = cam.rgb_width / 2
    middle_y = cam.rgb_height / 2

    # Referee commands
    ref_enabled = True
    if ref_enabled:
        referee = ref_cmd.Referee_cmd_client(log)
        ref_first_start = True
        robot_name = "OWO"
        referee.open()
    if (not ref_enabled) and state == State.START_WAIT:
        log.LOGE("Referee not enabled, but initial state is START_WAIT")

    if manualcontrol:
        state = State.MANUAL
        xboxcont = gamepad.gamepad(file = '/dev/input/event12')


    try:
        # Do not add anything outside of if/elif clauses to the end of the loop, otherwise use of "continue" will not let it run
        while(True):
            # Getting camera data
            if state == State.BALL_THROW:
                processedData = processor.process_frame(aligned_depth=True)
            else:
                processedData = processor.process_frame(aligned_depth=False)

            # Housekeeping stuff
            frame_cnt +=1
            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                log.LOGI("FPS: {}, framecount: {}".format(fps, frame_cnt))
                log.LOGI("ball_count: {}".format(len(processedData.balls)))
            # End of housekeeping stuff

            # Debug frame (turn off when over ssh)
            if debug:
                debug_frame = processedData.debug_frame
                cv2.imshow('debug', debug_frame)
                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
            # End of housekeeping

            # Referee command handling
            if ref_enabled:
                msg = referee.get_cmd()
                if msg is not None:
                    if robot_name in msg["targets"]:
                        if msg["signal"] == "start":
                            log.LOGI("Start signal received")
                            # ref_first_start used if we want to differentiate between start of the match and resuming from a stop
                            if ref_first_start:
                                log.LOGI("Match started!")
                                ref_first_start = False
                                state = State.START_GO
                                if msg["baskets"][msg["targets"].index("OWO")] == 'blue':
                                    log.LOGI("Blue basket selected")
                                    basket_color = TargetBasket.BLUE
                                elif msg["baskets"][msg["targets"].index("OWO")] == 'magenta':
                                    log.LOGI("Magenta basket selected")
                                    basket_color = TargetBasket.MAGENTA
                                else:
                                    log.LOGE("Basket color error")
                            else:
                                state = State.START_GO
                        elif msg["signal"] == "stop":
                            log.LOGI("Paused signal received")
                            state = State.PAUSED
            # End of referee commands

            # Main control logic uses a state machine
            # Inactive states
            if state == State.WAIT:
                log.LOGSTATE("waiting")
                input() # For making a break
                state = State.BALL_SEARCH
                continue

            if state == State.PAUSED:
                continue
            # End of inactive states

            # States used for thrower calibration
            elif state == State.CALIB_INPUT:
                thrower_speed = int(input("Speed:")) # Input speed for thrower
                state = State.CALIB_THROW

            elif state == State.CALIB_THROW: # Spin the motor for 10 seconds with the speed given in state "input"
                if calib_first_time:
                    calib_first_time = False
                    start_time = time.perf_counter()

                if (time.perf_counter() - start_time) > 10:
                    print("Average:", np.average(calibration_data))
                    print("Speed:", thrower_speed)

                    state = State.CALIB_INPUT
                    calib_first_time = True
                    calibration_data = []
                    continue

                if (processedData.basket_m.exists):
                    print("Distance:", processedData.basket_m.distance)
                    calibration_data.append(processedData.basket_m.distance)
                    robot.throw_raw(thrower_speed)
                else: 
                    print("No basket")
                    continue
            # End of states used for thrower calibration

            elif state == State.START_WAIT:
                continue

            elif state == State.START_GO:
                robot.move(0, 10, 0)
                if start_go_first_time:
                    log.LOGI("Juggernaut start")
                    start_go_time_start = time.perf_counter()
                    start_go_first_time = False
                if (start_go_time_start + 0.7 < time.perf_counter()):
                    log.LOGI("Juggernaut end")
                    state = State.BALL_SEARCH
                continue

            elif state == State.BALL_SEARCH:
                log.LOGSTATE("ball_search")
                if len(processedData.balls) > 0:
                    state = State.BALL_MOVE
                    continue
                else:
                    if (int(time.perf_counter() * 6) % 3 == 0):
                        robot.move(0, 0, 25)
                    else:
                        robot.move(0, 0, 3)
            # End of ball_search
            
            # Moving towards ball
            elif state == State.BALL_MOVE:
                log.LOGSTATE("ball_move")
                if len(processedData.balls) > 0:
                    # Movement logic
                    speed_x = 0
                    speed_y = 0
                    speed_r = 0

                    # Choosing the closest ball
                    interesting_ball = processedData.balls[-1]
                    #print("Ball:", interesting_ball)

                    if interesting_ball.distance <= 475:
                        state = State.BALL_ORBIT
                        continue
                    else:
                        if interesting_ball.x > middle_x + 2 or interesting_ball.x < middle_x - 2:
                            speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=1700, y_scale=max_speed/2)
                            speed_r = -sigmoid_controller(interesting_ball.x, middle_x, x_scale=1000, y_scale=max_speed)
                        if interesting_ball.distance > ball_good_range:
                            speed_y = sigmoid_controller(interesting_ball.distance, ball_good_range, x_scale=1400, y_scale=max_speed)
                        #print(f"x: {speed_x}, y: {speed_y}, r: {speed_r}, dist: {interesting_ball.distance}, b.x: {interesting_ball.x}, b.y: {interesting_ball.y}")
                        robot.move(speed_x, speed_y, speed_r)
                else:
                    state = State.BALL_SEARCH
                    continue
            # End of ball_move

            # Orbiting around ball until correct basket is found
            elif state == State.BALL_ORBIT:
                log.LOGSTATE("ball_orbit")
                if len(processedData.balls) > 0:
                    interesting_ball = processedData.balls[-1]

                    # For checking if the ball is still in position
                    if interesting_ball.distance > 550:
                        log.LOGE("Invalid radius, radius: " + str(interesting_ball.distance))
                        state = State.BALL_SEARCH
                        continue

                    # Determining the correct basket
                    if basket_color == TargetBasket.MAGENTA:
                        basket = processedData.basket_m
                    elif basket_color == TargetBasket.BLUE:
                        basket = processedData.basket_b
                    else:
                        log.LOGE("Basket color invalid")

                    if basket.exists:
                        # print("Basket x:", basket.x, "Middle x: ", middle_x)
                        basket_tolerance = 14
                        if abs(basket.x - middle_x) < basket_tolerance:
                            state = State.BALL_THROW
                            thrower_time_start = time.perf_counter()
                            continue
                        speed_x = -sigmoid_controller(basket.x, middle_x, x_scale=1500, y_scale=(max_speed - 3))

                        robot.orbit(400, speed_x, interesting_ball.distance, interesting_ball.x)

                    else:
                        robot.orbit(400, 2.8, interesting_ball.distance, interesting_ball.x)
                else:
                    log.LOGW("Lost ball during orbit")
                    state = State.BALL_SEARCH
                    continue
            # End of ball_orbit

            elif state == State.BALL_THROW:
                log.LOGSTATE("ball_throw")

                if basket_color == TargetBasket.MAGENTA:
                    basket = processedData.basket_m
                elif basket_color == TargetBasket.BLUE:
                    basket = processedData.basket_b
                else:
                    log.LOGE("Basket color invalid")

                if len(processedData.balls) > 0:
                    interesting_ball = processedData.balls[-1]

                if (thrower_time_start + 2.0 < time.perf_counter()):
                    log.LOGI("THROW, distance: " + str(basket.distance))
                    state = State.BALL_SEARCH

                log.LOGI(" Basket.x: " + str(basket.x) + " ball.x " + str(interesting_ball.x) + " ball.distance: " + str(interesting_ball.distance))
                speed_rot = -sigmoid_controller(basket.x, middle_x, x_scale=300, y_scale=max_speed)
                if (len(processedData.balls) != 0) and interesting_ball.distance > 300 and interesting_ball.distance < 600:
                    speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=500, y_scale=max_speed)
                else:
                    speed_x = 0

                speed_y = 1.5

                if not np.isnan(basket.distance):
                    robot.move(speed_x, speed_y, speed_rot, int(basket.distance))
                else: 
                    robot.move(speed_x, speed_y, speed_rot, 0)
                continue
            # End of ball_throw

            elif state == State.MANUAL:
                log.LOGSTATE("manual control")
                if xboxcont is None:
                    log.LOGE("gamepad is None, you entered this state incorrectly")
                    continue
                xboxcont.read_gamepad_input()
                joyY = 0
                joyX = 0
                joyRightX = 0
                joyRTrig = 0
                deadzone = 0.15

                if xboxcont.joystick_left_y > deadzone or xboxcont.joystick_left_y < -deadzone:
                    joyY = xboxcont.joystick_left_y

                if xboxcont.joystick_left_x > deadzone or xboxcont.joystick_left_x < -deadzone:
                    joyX = xboxcont.joystick_left_x

                if xboxcont.joystick_right_x > deadzone or xboxcont.joystick_right_x < -deadzone:
                    joyRightX = -xboxcont.joystick_right_x
                
                joyRTrig = xboxcont.trigger_right

                #print(str(xboxcont.joystick_left_y) + " / " + str(xboxcont.joystick_left_x))

                speedy = joyY * 5
                speedx = joyX * 5
                speedr = joyRightX * 20
                speedthrow = joyRTrig * 1200

                print("Y: " + str(speedy) + " X: " + str(speedx) + " R: " + str(speedr))

                robot.move(speedx, speedy, speedr, speedthrow)
                

            else: # Unknown state
                # Considered as an error
                log.LOGSTATE("unknown state")
                log.LOGE("Exiting...")
                break
            # End of unknown

    except KeyboardInterrupt:
        print("\nExiting")
    
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        if ref_enabled:
            referee.close()
        robot.close()
        log.close()
        sys.exit()
