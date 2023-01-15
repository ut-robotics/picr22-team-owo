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

class Target_basket(Enum):
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
    # Timeout states
    TIMEOUT = 12
    # New robot states
    BALL_EAT = 13
    BASKET_FIND = 14

class Search_state(Enum):
    ROTATE_SLOW = 1
    ROTATE_FAST = 2

if __name__ == "__main__":
    # Housekeeping setup
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    log = Logging(True, True)
    log.LOGI("Starting...")

    # Mainboard stuff setup
    max_speed = 20
    max_speed_inner = 30
    ball_good_range = 450
    robot = mainboard.Mainboard(max_speed_inner, log)
    robot.start()

    # Control logic setup
    debug = False
    state = State.START_WAIT # <====================================================== Initial state here!
    thrower_speed = 0
    calib_first_time = True
    calibration_data = []
    basket_color = Target_basket.MAGENTA # currently defaults to magenta for testing purposes
    thrower_time_start = 0
    start_go_time_start = 0
    start_go_first_time = True

    # Ball search
    search_state = Search_state.ROTATE_SLOW
    ball_search_first_time = True

    # Vision setup
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()

    # Manual control
    xbox_cont = None
    manual_control = True

    # Constants etc.
    # Housekeeping
    has_thrown = False
    manual_triggers = 0  # for detecting a single button press
    basket_change_triggers = 0 # for detecting a single button press

    # Camera constants
    middle_x = cam.rgb_width / 2
    middle_y = cam.rgb_height / 2

    # Throwing
    throw_angle_chosen = False
    last_known_basket_distance = 0
    throw_ending_in_progress = False
    throw_ending_start_time = 0
    throw_ending_delay = 0.8
    throwing = False
    throw_check_counter = 0

    # Referee commands
    ref_enabled = False
    if ref_enabled:
        robot_name = "owo"
        referee = ref_cmd.Referee_cmd_client(log)
        ref_first_start = True
        referee.open()
    if (not ref_enabled) and state == State.START_WAIT:
        log.LOGE("Referee not enabled, but initial state is START_WAIT")

    
    try:
        xbox_cont = gamepad.Gamepad(file = '/dev/input/event12')
    except FileNotFoundError:
        log.LOGW("Controller not connected")


    try:
        # Initializing the previous-state variable
        prev_state = State.WAIT
        # Do not add anything outside of if/elif state clauses to the end of the loop, otherwise use of "continue" will not let it run
        while(True):
            # Getting camera data
            if state in (State.BALL_THROW, State.BASKET_FIND, State.BALL_EAT, State.CALIB_THROW, State.BALL_MOVE):
                processed_data = processor.process_frame(aligned_depth=True)
            else:
                processed_data = processor.process_frame(aligned_depth=False)

            if prev_state != state:
                state_start_timestamp = time.perf_counter()
                prev_state = state

            # Manual Controller managing
            if manual_control and xbox_cont is not None:
                xbox_cont.read_gamepad_input()

                if state == State.WAIT:
                    state = State.MANUAL
                
                if xbox_cont.button_b:
                    log.LOGE("Code stopped by manual kill switch..")
                    break

                # For changing between manual and autonomous modes
                # This trickery required because of controller API
                if not xbox_cont.button_x and manual_triggers >= 2:
                    manual_triggers = 0

                elif xbox_cont.button_x and state != State.MANUAL and manual_triggers == 0:
                    state = State.MANUAL
                    manual_triggers += 1
                    continue

                elif xbox_cont.button_x and state == State.MANUAL and manual_triggers == 0:
                    state = State.BALL_SEARCH
                    manual_triggers += 1
                    continue
                
                elif xbox_cont.button_x:
                    manual_triggers += 1

                # For changing target basket
                if not xbox_cont.button_y and basket_change_triggers >= 2:
                    basket_change_triggers = 0

                elif xbox_cont.button_y and basket_change_triggers == 0:
                    if basket_color == Target_basket.BLUE:
                        basket_color = Target_basket.MAGENTA
                    elif basket_color == Target_basket.MAGENTA:
                        basket_color = Target_basket.BLUE
                    basket_change_triggers += 1

                elif xbox_cont.button_y:
                    basket_change_triggers += 1


            # Housekeeping stuff
            frame_cnt +=1
            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                log.LOGI("FPS: {}, framecount: {}".format(fps, frame_cnt))
                log.LOGI("ball_count: {}".format(len(processed_data.balls)))
            # End of housekeeping stuff

            # Debug frame (turn off when over ssh)
            if debug:
                debug_frame = processed_data.debug_frame
                cv2.imshow('debug', debug_frame)
                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    log.LOGI("q pressed on debug screen")
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
                            if msg["baskets"][msg["targets"].index(robot_name)] == 'blue':
                                log.LOGI("Blue basket selected")
                                basket_color = Target_basket.BLUE
                            elif msg["baskets"][msg["targets"].index(robot_name)] == 'magenta':
                                log.LOGI("Magenta basket selected")
                                basket_color = Target_basket.MAGENTA
                            else:
                                log.LOGE("Basket color error")
                            if ref_first_start:
                                log.LOGI("Match started!")
                                ref_first_start = False
                                state = State.BALL_SEARCH
                                start_go_time_start = time.perf_counter()
                            else:
                                log.LOGI("Match resumed!")
                                state = State.BALL_SEARCH
                                start_go_time_start = time.perf_counter()
                        elif msg["signal"] == "stop":
                            log.LOGI("Match paused!")
                            robot.stop_all()
                            state = State.PAUSED

            # End of referee commands
            
            # ----- NEW ROBOT BALL-IN SENSOR HANDLING -----
            
            # If a ball is currently in the belly of the beast, find a basket
            if robot.ball_in_robot and state in (State.BALL_SEARCH, State.BALL_MOVE, State.BALL_EAT):
                robot.eating_servo(mainboard.Eating_servo_state.OFF)
                state = State.BASKET_FIND
                robot.eating_servo(mainboard.Eating_servo_state.OFF)

            # Main control logic uses a state machine
            # Inactive states
            if state == State.WAIT:
                log.LOGSTATE("waiting")
                input() # For making a break
                state = State.BALL_SEARCH
                continue

            if state == State.PAUSED:
                log.LOGSTATE("paused")
                continue
            # End of inactive states

            elif state == State.TIMEOUT:
                log.LOGSTATE("timeout")
                timeout_speed_x = 2
                timeout_speed_y = 5
                timeout_speed_rot = 0
                timeout_duration = 0.5
                if time.perf_counter() > state_start_timestamp + timeout_duration:
                    state = State.BALL_SEARCH
                    continue

                if basket.exists:
                    if basket.distance < 500:
                        timeout_speed_y = -5
                        timeout_speed_rot = 15

                robot.move(timeout_speed_x, timeout_speed_y, timeout_speed_rot)

            # States used for thrower calibration
            elif state == State.CALIB_INPUT:
                thrower_speed = int(input("Speed:")) # Input speed for thrower
                state = State.CALIB_THROW

            elif state == State.CALIB_THROW: # Spin the motor for 10 seconds with the speed given in state "input"
                robot.eating_servo(mainboard.Eating_servo_state.EAT)
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

                if (processed_data.basket_m.exists):
                    print("Distance:", processed_data.basket_m.distance)
                    calibration_data.append(processed_data.basket_m.distance)
                    robot.throw_raw(thrower_speed, robot.succ_servo_in, robot.angle_servo_high)
                else: 
                    print("No basket")
                    continue
            # End of states used for thrower calibration

            elif state == State.START_WAIT:
                log.LOGSTATE("waiting for start")
                continue

            elif state == State.START_GO:
                robot.move(0, 20, 0)
                log.LOGSTATE("Juggernaut start")

                if start_go_first_time:
                    start_go_first_time = False
                    start_go_time_start = time.perf_counter()

                if (start_go_time_start + 0.7 < time.perf_counter()):
                    log.LOGI("Juggernaut end")
                    state = State.BALL_SEARCH
                    start_go_first_time = True
                continue

            elif state == State.BALL_SEARCH:
                log.LOGSTATE("ball_search")
                if len(processed_data.balls) > 0:
                    ball_search_first_time = True
                    state = State.BALL_MOVE
                    continue

                if search_state == Search_state.ROTATE_SLOW:
                    if ball_search_first_time:
                        ball_search_first_time = False
                        ball_search_start = time.perf_counter()
                    if time.perf_counter() > ball_search_start + 0.3:
                        search_state = Search_state.ROTATE_FAST
                    robot.move(0, 0, -5)
                elif search_state == Search_state.ROTATE_FAST:
                    if time.perf_counter() > ball_search_start + 0.3 + 0.2:
                        ball_search_first_time = True
                        search_state = Search_state.ROTATE_SLOW
                    robot.move(0, 0, -50)
                else:
                    log.LOGE("Invalid search_state, defaulting to ROTATE_SLOW")
                    search_state = Search_state.ROTATE_SLOW
                    continue
            # End of ball_search
            
            # Moving towards ball
            elif state == State.BALL_MOVE:
                log.LOGSTATE("ball_move")
                if len(processed_data.balls) > 0:
                    # Movement logic
                    speed_x = 0
                    speed_y = 0
                    speed_r = 0

                    # Choosing the closest ball
                    interesting_ball = processed_data.balls[-1]

                    if interesting_ball.distance <= 550 and (abs(middle_x - interesting_ball.x) < 80):
                        state = State.BALL_EAT
                        throw_check_counter = 0
                        continue
                    else:
                        speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=1400, y_scale=max_speed/2)
                        speed_r = -sigmoid_controller(interesting_ball.x, middle_x, x_scale=1000, y_scale=max_speed)
                        speed_y = sigmoid_controller(interesting_ball.distance, ball_good_range, x_scale=1000, y_scale=max_speed)
                        if speed_y == 0:
                            speed_y = 5
                        robot.move(speed_x, speed_y, speed_r)
                else:
                    state = State.BALL_SEARCH
                    continue
            # End of ball_move
            
            # Ball eating state
            elif state == State.BALL_EAT:
                log.LOGSTATE("ball_eat")
                # Maximum time to stay in this state, in seconds.
                max_time_in_ball_eat = 0.65
                # Speed for eating servo, units undetermined, current value temporary
                eat_servo_speed = 10

                

                if state_start_timestamp + max_time_in_ball_eat > time.perf_counter():
                    # Default movement values
                    speed_x = 0 
                    speed_y = 6 # <--- Change this to change the forward movement speed of this state
                    speed_r = 0
                    interesting_ball = None
                    
                    robot.eating_servo(mainboard.Eating_servo_state.EAT)
                    
                    if len(processed_data.balls) > 0:
                        interesting_ball = processed_data.balls[-1]
                        if interesting_ball.distance > 550:
                            interesting_ball = None # Could use some kind of previous value here?
                        
                    if interesting_ball != None:
                        speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=900, y_scale=max_speed/2)
                        speed_r = -sigmoid_controller(interesting_ball.x, middle_x, x_scale=500, y_scale=max_speed*1.5)
                        robot.move(speed_x, speed_y, speed_r)
                    else:
                        robot.move(0, speed_y, 0)
                    
                    continue
                
                else:
                    log.LOGE("timeout from eat")
                    state = State.BALL_SEARCH
                    continue
            # end of ball eating
            
            # State of the new robot for finding the basket when ball is eaten
            elif state == State.BASKET_FIND:
                timeout_trig_basket_find = 4
                if time.perf_counter() > state_start_timestamp + timeout_trig_basket_find:
                    state = State.TIMEOUT
                    continue
                log.LOGSTATE("basket_find")
                
                # Default movement values
                speed_x = 0 
                speed_y = 0 
                speed_r = -30  # <--- Change this to change the basket finding rotation speed
                    
                # Determining the correct basket
                if basket_color == Target_basket.MAGENTA:
                    basket = processed_data.basket_m
                elif basket_color == Target_basket.BLUE:
                    basket = processed_data.basket_b
                else:
                    log.LOGE("Basket color invalid")
                        
                # distance in mm, closer than this means moving backwards, else move forward
                basket_decision_dist = 1200
                
                # If basket is found, go to throwing, NB! Basket centering done entirely in throw
                if basket.exists:
                    state = State.BALL_THROW
                    throwing = False
                    throw_check_counter = 0
                    if basket.distance < basket_decision_dist:
                        robot.driving_forward = False
                    else:
                        robot.driving_forward = True

                    continue
                # Need to check basket distance here, so that wouldn't be too close or too far away
                        
                robot.move(speed_x, speed_y, speed_r)
                continue

            elif state == State.BALL_THROW:
                log.LOGSTATE("ball_throw")
                time_in_throw = 2.5

                if basket_color == Target_basket.MAGENTA:
                    basket = processed_data.basket_m
                elif basket_color == Target_basket.BLUE:
                    basket = processed_data.basket_b
                else:
                    log.LOGE("Basket color invalid")

                if not basket.exists:
                    if robot.ball_in_robot:
                        state = State.BASKET_FIND
                    else:
                        state = State.BALL_SEARCH
                    continue

                if basket.distance < 600 and robot.driving_forward:
                    robot.driving_forward = False

                if basket.distance > 2000 and not robot.driving_forward:
                    robot.driving_forward = True

                if throw_ending_in_progress and throw_ending_start_time + throw_ending_delay < time.time():
                    # End throw state for new robot, back to ball search
                    robot.eating_servo(mainboard.Eating_servo_state.OFF)
                    throw_angle_chosen = False
                    throw_ending_in_progress = False
                    state = State.BALL_SEARCH
                    continue
                
                speed_x = 0

                # IMPORTANT
                throw_required_correct_frames = 5

                if not robot.ball_in_robot and not throw_ending_in_progress:
                    log.LOGI("Ball has left the sensor, starting countdown to end throw...")
                    throw_ending_start_time = time.perf_counter()
                    throw_ending_in_progress = True
                else:
                    # Default movement values
                    speed_r = 0
                    maximum_basket_error = 6

                    if basket.distance > 2000:
                        maximum_basket_error = 3

                    if robot.driving_forward:
                        if basket.distance > robot.throw_radius_max:
                            throw_move_speed = 10
                        else:
                            throw_move_speed = 2
                    else:
                        if basket.distance < robot.throw_radius_min:
                            throw_move_speed = -10
                        else:
                            throw_move_speed = -2
                    
                    # Speed calculations
                    log.LOGI(" Basket.x: " + str(basket.x))
                    
                    # If basket is in the middle of the screen.
                    if (abs(middle_x - basket.x) <= maximum_basket_error and basket.distance >= robot.throw_radius_min and basket.distance <= robot.throw_radius_max) or throwing:
                        if not throwing:
                            throw_check_counter += 1

                        if throw_check_counter >= throw_required_correct_frames or throwing:

                            throwing = True # Commited to making a throw, stuck here until ball has left the sensor
                            robot.eating_servo(mainboard.Eating_servo_state.EAT)

                            # Smoother correction still in place just in case.
                            speed_r = -sigmoid_controller(basket.x, middle_x, x_scale=500, y_scale=max_speed)

                            # Worth considering - saving last known basket distance. ++DONE++
                            if not np.isnan(basket.distance) and basket.exists:
                                robot.move(0, throw_move_speed, speed_r, int(basket.distance))
                                last_known_basket_distance = int(basket.distance)
                            else: 
                                robot.move(0, throw_move_speed, speed_r, last_known_basket_distance)
                            
                    # This means basket needs centering
                    else:
                        throw_check_counter = 0
                        speed_r = -sigmoid_controller(basket.x, middle_x, x_scale=300, y_scale=max_speed)
                        if not np.isnan(basket.distance):
                            robot.choose_thrower_angle(basket.distance)
                            throw_angle_chosen = True
                        if not np.isnan(basket.distance) and basket.exists:
                            robot.move(0, throw_move_speed, speed_r, int(basket.distance))
                            last_known_basket_distance = int(basket.distance)
                        else: 
                            robot.move(0, throw_move_speed, speed_r, last_known_basket_distance)
                        
                continue
            # End of ball_throw

            elif state == State.MANUAL:
                log.LOGSTATE("manual control")
                if xbox_cont is None:
                    log.LOGE("gamepad is None, you entered this state incorrectly")
                    continue
                xbox_cont.read_gamepad_input()
                joy_y = 0
                joy_x = 0
                joy_right_x = 0
                joy_r_trig = 0
                deadzone = 0.15

                if xbox_cont.joystick_left_y > deadzone or xbox_cont.joystick_left_y < -deadzone:
                    joy_y = xbox_cont.joystick_left_y

                if xbox_cont.joystick_left_x > deadzone or xbox_cont.joystick_left_x < -deadzone:
                    joy_x = xbox_cont.joystick_left_x

                if xbox_cont.joystick_right_x > deadzone or xbox_cont.joystick_right_x < -deadzone:
                    joy_right_x = -xbox_cont.joystick_right_x
                
                joy_r_rig = xbox_cont.trigger_right

                speed_y = joy_y * 13
                speed_x = joy_x * 13
                speed_r = joy_right_x * 30
                speed_throw = joy_r_trig * 4500

                if joy_r_trig > 0  :
                    robot.eating_servo(mainboard.Eating_servo_state.EAT)
                else:
                    robot.eating_servo(mainboard.Eating_servo_state.OFF)

                robot.move(speed_x, speed_y, speed_r, speed_throw)
                

            else: # Unknown state
                # Considered as an error
                log.LOGSTATE("unknown state")
                log.LOGE("Exiting...")
                break
            # End of unknown

    except KeyboardInterrupt:
        print("\nExiting")

    except Exception as e:
        print(e)
    
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        if ref_enabled:
            referee.close()
        robot.close()
        log.close()
        sys.exit()
