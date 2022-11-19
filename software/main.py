#!/usr/bin/python3

# Python
import time, sys, math, argparse
from enum import Enum

# Our code
import gamepad
import mainboard
from robot_utilities import *
import image_processor
import camera
import ref_cmd
import config_parser

# External dependencies
import cv2

# Argparser: ref_ip, ref_port, start_state, config_file, basket, 
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

if __name__ == "__main__":
    # Command line arguments
    parser = argparse.ArgumentParser()
    # Options for overriding default (config file) values for config file name, start state, referee server ip and port, basket color
    # If the command line argument is provided it always overrides the config file value
    parser.add_argument("-c", "--conf_file", help="config file name, do not add config folder to the name")
    parser.add_argument("-s", "--start_state", help="State in which the robots starts", type=int)
    parser.add_argument("--ip", help="ip adress of referee server")
    parser.add_argument("--port", help="port of the referee server")
    parser.add_argument("-b", "--basket", help="Into which basket robot throws balls", choices=["blue", "magenta"])
    parser.add_argument("-d", "--debug", help="Debug mode active or not", action="store_true")

    args = parser.parse_args()

    # Config
    if args.conf_file is None:
        config = config_parser.Config_parser("default_config.toml")
    else:
        config = config_parser.Config_parser(args.conf_file)
    
    config.parse()
    if len(config.data) == 0:
        # Reading the config file faileds
        print("Reading the config failed")
        sys.exit()

    # Housekeeping setup
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    log = Logging(config)
    log.LOGI("Starting...")

    # Logic setup
    logic_conf = config.get_logic_dict()
    robot_name = logic_conf["robot_name"]
    manual_enabled = logic_conf["manual_enabled"]
    referee_enabled = logic_conf["referee_enabled"]
    max_speed = logic_conf["max_speed"]
    thrower_time_start = 0
    start_go_time_start = 0
    start_go_first_time = True
    # Starting state
    if args.start_state is None:
        state = State(logic_conf["start_state"])
    else:
        state = State(args.start_state)
    # Debug
    if args.debug is None:
        debug = logic_conf["debug"]
    else:
        debug = args.debug
    # Basket
    if args.basket is None:
        basket_color = Target_basket(logic_conf["basket"])
    else:
        if args.basket == "blue":
            basket_color = Target_basket.BLUE
        elif args.basket == "magenta":
            basket_color = Target_basket.MAGENTA

    # Mainboard stuff setup
    robot = mainboard.Mainboard(config, log)
    robot.start()

    # Vision setup
    cam = camera.RealsenseCamera(config)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()
    middle_x = cam.rgb_width / 2
    middle_y = cam.rgb_height / 2

    # Manual control setup
    xbox_cont = None
    manual_triggers = 0
    try:
        xbox_cont = gamepad.Gamepad(config)
    except FileNotFoundError:
        log.LOGW("Controller not connected")

    # Referee commands
    if referee_enabled:
        referee = ref_cmd.Referee_cmd_client(log)
        ref_first_start = True
        referee.open()
    if (not referee_enabled) and state == State.START_WAIT:
        log.LOGE("Referee not enabled, but initial state is START_WAIT")

    # State machine setup
    ball_good_range = 300
    has_thrown = False


    # Calibration setup
    thrower_speed = 0
    calib_first_time = True
    calibration_data = []

    try:
        # Do not add anything outside of if/elif state clauses to the end of the loop, otherwise use of "continue" will not let it run
        while(True):
            # Getting camera data
            if state == State.BALL_THROW:
                processed_data = processor.process_frame(aligned_depth=True)
            else:
                processed_data = processor.process_frame(aligned_depth=False)

            # Manual Controller managing
            if xbox_cont is not None and manual_enabled:
                xbox_cont.read_gamepad_input()

                if xbox_cont.button_b:
                    log.LOGE("Code stopped by manual kill switch..")
                    break

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
                    break
            # End of housekeeping

            # Referee command handling
            if referee_enabled:
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
                                state = State.START_GO
                                start_go_time_start = time.perf_counter()
                            else:
                                log.LOGI("Match resumed!")
                                state = State.START_GO
                                start_go_time_start = time.perf_counter()
                        elif msg["signal"] == "stop":
                            log.LOGI("Match paused!")
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

                if (processed_data.basket_m.exists):
                    print("Distance:", processed_data.basket_m.distance)
                    calibration_data.append(processed_data.basket_m.distance)
                    robot.throw_raw(thrower_speed)
                else: 
                    print("No basket")
                    continue
            # End of states used for thrower calibration

            elif state == State.START_WAIT:
                log.LOGSTATE("Start wait")
                continue

            elif state == State.START_GO:
                robot.move(0, 10, 0)
                #if start_go_first_time:
                log.LOGSTATE("Juggernaut")
                    
                    #start_go_first_time = False
                if (start_go_time_start + 0.6 < time.perf_counter()): # Yandalf - changed from 0.7 to 0.6
                    log.LOGI("Juggernaut end")
                    state = State.BALL_SEARCH
                continue

            elif state == State.BALL_SEARCH:
                log.LOGSTATE("ball_search")
                if len(processed_data.balls) > 0:
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
                if len(processed_data.balls) > 0:
                    # Movement logic
                    speed_x = 0
                    speed_y = 0
                    speed_r = 0

                    # Choosing the closest ball
                    interesting_ball = processed_data.balls[-1]
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
                if len(processed_data.balls) > 0:
                    interesting_ball = processed_data.balls[-1]

                    # For checking if the ball is still in position
                    if interesting_ball.distance > 550:
                        log.LOGE("Invalid radius, radius: " + str(interesting_ball.distance))
                        state = State.BALL_SEARCH
                        continue

                    # Determining the correct basket
                    if basket_color == Target_basket.MAGENTA:
                        basket = processed_data.basket_m
                    elif basket_color == Target_basket.BLUE:
                        basket = processed_data.basket_b
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

                if basket_color == Target_basket.MAGENTA:
                    basket = processed_data.basket_m
                elif basket_color == Target_basket.BLUE:
                    basket = processed_data.basket_b
                else:
                    log.LOGE("Basket color invalid")

                if len(processed_data.balls) > 0:
                    interesting_ball = processed_data.balls[-1]

                if (thrower_time_start + 2.0 < time.perf_counter()):
                    log.LOGI("THROW, distance: " + str(basket.distance))
                    state = State.BALL_SEARCH

                log.LOGI(" Basket.x: " + str(basket.x) + " ball.x " + str(interesting_ball.x) + " ball.distance: " + str(interesting_ball.distance))
                speed_rot = -sigmoid_controller(basket.x, middle_x, x_scale=300, y_scale=max_speed)
                if (len(processed_data.balls) != 0) and interesting_ball.distance > 250 and interesting_ball.distance < 500:
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
                if xbox_cont is None:
                    log.LOGE("gamepad is None, you entered this state incorrectly")
                    continue
                xbox_cont.read_gamepad_input()
                joyY = 0
                joyX = 0
                joyRightX = 0
                joyRTrig = 0
                deadzone = 0.15

                if xbox_cont.joystick_left_y > deadzone or xbox_cont.joystick_left_y < -deadzone:
                    joyY = xbox_cont.joystick_left_y

                if xbox_cont.joystick_left_x > deadzone or xbox_cont.joystick_left_x < -deadzone:
                    joyX = xbox_cont.joystick_left_x

                if xbox_cont.joystick_right_x > deadzone or xbox_cont.joystick_right_x < -deadzone:
                    joyRightX = -xbox_cont.joystick_right_x
                
                joyRTrig = xbox_cont.trigger_right

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
        if referee_enabled:
            referee.close()
        robot.close()
        log.close()
        sys.exit()
