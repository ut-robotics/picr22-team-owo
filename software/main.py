#!/usr/bin/python3

# Python
import time, sys, argparse
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
    BALL_PATROL = 6
    BALL_MOVE = 7
    BALL_ORBIT = 8
    BALL_THROW = 9
    # These 2 are for thrower calibration
    CALIB_INPUT = 10
    CALIB_THROW = 11
    # Control with XBox controller
    MANUAL = 12
    # Timeout state
    TIMEOUT = 13

class Search_state(Enum):
    ROTATE_SLOW = 1
    ROTATE_FAST = 2

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
        # Reading the config file failed
        print("Reading the config failed")
        sys.exit()

    # Housekeeping setup
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    log = Logging(config)
    log.LOGI("Starting...")
    state_start_timestamp = time.time()

    # Logic setup
    logic_conf = config.get_logic_dict()
    robot_name = logic_conf["robot_name"]
    manual_enabled = logic_conf["manual_enabled"]
    referee_enabled = logic_conf["referee_enabled"]
    max_speed = logic_conf["max_speed"]
    thrower_time_start = 0
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
    # start_go
    juggernaut_time = config.get_state_dict("start_go")["juggernaut_time"]
    start_go_time_start = 0
    start_go_first_time = True
    # ball_search
    ball_search_first_time = True
    ball_search_ball_time = 0
    search_counter = 0
    search_state = Search_state.ROTATE_SLOW
    search_range_enabled = True
    ball_search_dict = config.get_state_dict("ball_search")
    ball_search_count_patrol = ball_search_dict["count_patrol"]
    ball_search_slow_time = ball_search_dict["slow_time"]
    ball_search_fast_time = ball_search_dict["fast_time"]
    rotation_speed_slow = ball_search_dict["rotation_speed_slow"]
    rotation_speed_fast = ball_search_dict["rotation_speed_fast"]
    # ball_patrol
    ball_patrol_dict = config.get_state_dict("ball_patrol")
    patrol_active_basket = basket_color
    patrol_x_speed_y_multiplier = ball_patrol_dict["x_speed_y_multiplier"]
    patrol_r_speed_y_multiplier = ball_patrol_dict["r_speed_y_multiplier"]
    patrol_x_speed_x_scale = ball_patrol_dict["x_speed_x_scale"]
    patrol_r_speed_x_scale = ball_patrol_dict["r_speed_x_scale"]
    patrol_y_speed = ball_patrol_dict["y_speed"]
    patrol_range = ball_patrol_dict["range"]
    # ball_move
    ball_move_dict = config.get_state_dict("ball_move")
    distance_into_orbit = ball_move_dict["distance_into_orbit"]
    distance_ball_good_range = ball_move_dict["distance_ball_good_range"]
    x_speed_x_scale = ball_move_dict["x_speed_x_scale"]
    r_speed_x_scale = ball_move_dict["r_speed_x_scale"]
    y_speed_x_scale = ball_move_dict["y_speed_x_scale"]
    x_speed_y_multiplier = ball_move_dict["x_speed_y_multiplier"]
    r_speed_y_multiplier = ball_move_dict["r_speed_y_multiplier"]
    y_speed_y_multiplier = ball_move_dict["y_speed_y_multiplier"]
    # ball_orbit
    ball_orbit_dict = config.get_state_dict("ball_orbit")
    out_of_range_radius = ball_orbit_dict["out_of_range_radius"]
    orbit_radius = ball_orbit_dict["orbit_radius"]
    basket_x_tolerance = ball_orbit_dict["basket_x_tolerance"]
    x_speed_x_scale = ball_orbit_dict["x_speed_x_scale"]
    x_speed_y_multiplier = ball_orbit_dict["x_speed_y_multiplier"]
    no_basket_orbit_speed = ball_orbit_dict["no_basket_orbit_speed"]
    # ball_throw
    ball_throw_dict = config.get_state_dict("ball_throw")
    bt_throw_drive_time = ball_throw_dict["throw_drive_time"]
    bt_y_speed = ball_throw_dict["y_speed"]
    bt_r_speed_x_scale = ball_throw_dict["r_speed_x_scale"]
    bt_r_speed_y_multiplier = ball_throw_dict["r_speed_y_multiplier"]
    bt_x_speed_x_scale = ball_throw_dict["x_speed_x_scale"]
    bt_x_speed_y_multiplier = ball_throw_dict["x_speed_y_multiplier"]
    bt_ball_min_radius = ball_throw_dict["ball_min_radius"]
    bt_ball_max_radius = ball_throw_dict["ball_max_radius"]
    # manual
    manual_dict = config.get_state_dict("manual")
    m_deadzone = manual_dict["deadzone"]
    m_y_scaling = manual_dict["y_scaling"]
    m_x_scaling = manual_dict["x_scaling"]
    m_r_scaling = manual_dict["r_scaling"]
    m_thrower_scaling = manual_dict["thrower_scaling"]

    # Manual control state automatic switching when controller connected
    if xbox_cont != None:
        state = State.MANUAL

    # Calibration setup
    thrower_speed = 0
    calib_first_time = True
    calibration_data = []
    

    try:
        # Initialize previous state variable to the starting state
        prev_state = state
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
                
            if prev_state != state:
                state_start_timestamp = time.time()
                prev_state = state

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

            # State for triggered timeout from other states
            elif state == State.TIMEOUT:
                log.LOGSTATE("timeout")
                timeout_speed_x = 2
                timeout_speed_y = 5
                timeout_speed_rot = 0
                timeout_duration = 0.5
                if time.time() > state_start_timestamp + timeout_duration:
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
                if (start_go_time_start + juggernaut_time < time.perf_counter()): # Yandalf - changed from 0.7 to 0.6
                    log.LOGI("Juggernaut end")
                    state = State.BALL_SEARCH
                continue

            elif state == State.BALL_SEARCH:
                log.LOGSTATE("ball_search")

                if len(processed_data.balls) > 0:
                    closest_ball = processed_data.balls[-1]
                    search_counter = 0
                    state = State.BALL_MOVE
                    continue

                if search_counter > ball_search_count_patrol:
                    if basket_color == Target_basket.MAGENTA:
                        basket = processed_data.basket_m
                    elif basket_color == Target_basket.BLUE:
                        basket = processed_data.basket_b
                    else:
                        log.LOGE("Basket color invalid")

                    if basket.exists:
                        state = State.BALL_PATROL
                        search_counter = 0
                        ball_search_first_time = True
                        continue

                if search_state == Search_state.ROTATE_SLOW:
                    if ball_search_first_time:
                        search_counter += 1
                        ball_search_first_time = False
                        ball_search_start = time.perf_counter()
                    if time.perf_counter() > ball_search_start + ball_search_slow_time:
                        search_state = Search_state.ROTATE_FAST
                    robot.move(0, 0, rotation_speed_slow)
                elif search_state == Search_state.ROTATE_FAST:
                    if time.perf_counter() > ball_search_start + ball_search_slow_time + ball_search_fast_time:
                        ball_search_first_time = True
                        search_state = Search_state.ROTATE_SLOW
                    robot.move(0, 0, rotation_speed_fast)
                else:
                    log.LOGE("Invalid search_state, defaulting to ROTATE_SLOW")
                    search_state = Search_state.ROTATE_SLOW
                    continue

            elif state == State.BALL_PATROL:
                timeout_trig_patrol = 12
                if time.time() > state_start_timestamp + timeout_trig_move:
                    state = State.TIMEOUT
                    continue
                log.LOGSTATE("ball_patrol")

                if patrol_active_basket == Target_basket.MAGENTA:
                    basket = processed_data.basket_m
                elif patrol_active_basket == Target_basket.BLUE:
                    basket = processed_data.basket_b

                if basket.exists and basket.distance < patrol_range:
                    if patrol_active_basket == Target_basket.MAGENTA:
                        patrol_active_basket = Target_basket.BLUE
                    else:
                        patrol_active_basket = Target_basket.MAGENTA
                    state = State.BALL_SEARCH
                    robot.move(0,-5,0)
                    continue

                if len(processed_data.balls) > 0:
                    closest_ball = processed_data.balls[-1]
                    if patrol_active_basket == Target_basket.MAGENTA:
                        patrol_active_basket = Target_basket.BLUE
                    else:
                        patrol_active_basket = Target_basket.MAGENTA
                    state = State.BALL_MOVE
                    continue

                if basket.exists:
                    speed_x = sigmoid_controller(basket.x, middle_x, x_scale=patrol_x_speed_x_scale, y_scale=max_speed*patrol_x_speed_y_multiplier)
                    speed_r = -sigmoid_controller(basket.x, middle_x, x_scale=patrol_r_speed_x_scale, y_scale=max_speed*patrol_r_speed_y_multiplier)
                    speed_y = patrol_y_speed
                    robot.move(speed_x, speed_y, speed_r)
                else:
                    robot.move(0, 0, 15)
                
            # End of ball_search
            
            # Moving towards ball
            elif state == State.BALL_MOVE:
                timeout_trig_move = 10
                if time.time() > state_start_timestamp + timeout_trig_move:
                    state = State.TIMEOUT
                    continue
                log.LOGSTATE("ball_move")
                if len(processed_data.balls) > 0:
                    # Movement logic
                    speed_x = 0
                    speed_y = 0
                    speed_r = 0

                    # Choosing the closest ball
                    interesting_ball = processed_data.balls[-1]
                    #print("Ball:", interesting_ball)

                    if interesting_ball.distance <= distance_into_orbit:
                        state = State.BALL_ORBIT
                        continue
                    else:
                        if interesting_ball.x > middle_x + 2 or interesting_ball.x < middle_x - 2:
                            speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=x_speed_x_scale, y_scale=max_speed*x_speed_y_multiplier)
                            speed_r = -sigmoid_controller(interesting_ball.x, middle_x, x_scale=r_speed_x_scale, y_scale=max_speed*r_speed_y_multiplier)
                        if interesting_ball.distance > distance_ball_good_range:
                            speed_y = sigmoid_controller(interesting_ball.distance, ball_good_range, x_scale=y_speed_x_scale, y_scale=max_speed*y_speed_y_multiplier)
                        #print(f"x: {speed_x}, y: {speed_y}, r: {speed_r}, dist: {interesting_ball.distance}, b.x: {interesting_ball.x}, b.y: {interesting_ball.y}")
                        robot.move(speed_x, speed_y, speed_r)
                else:
                    state = State.BALL_SEARCH
                    continue
            # End of ball_move

            # Orbiting around ball until correct basket is found
            elif state == State.BALL_ORBIT:
                timeout_trig_orbit = 5
                if time.time() > state_start_timestamp + timeout_trig_orbit:
                    state = State.TIMEOUT
                    continue
                log.LOGSTATE("ball_orbit")
                if len(processed_data.balls) > 0:
                    interesting_ball = processed_data.balls[-1]

                    # For checking if the ball is still in position
                    if interesting_ball.distance > out_of_range_radius:
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
                        if abs(basket.x - middle_x) < basket_x_tolerance:
                            state = State.BALL_THROW
                            thrower_time_start = time.perf_counter()
                            continue
                        speed_x = -sigmoid_controller(basket.x, middle_x, x_scale=x_speed_x_scale, y_scale=max_speed*x_speed_y_multiplier)

                        robot.orbit(orbit_radius, speed_x, interesting_ball.distance, interesting_ball.x)

                    else:
                        robot.orbit(orbit_radius, no_basket_orbit_speed, interesting_ball.distance, interesting_ball.x)
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

                if (thrower_time_start + bt_throw_drive_time < time.perf_counter()):
                    log.LOGI("THROW, distance: " + str(basket.distance))
                    state = State.BALL_SEARCH

                #log.LOGI(" Basket.x: " + str(basket.x) + " ball.x " + str(interesting_ball.x) + " ball.distance: " + str(interesting_ball.distance))
                speed_rot = -sigmoid_controller(basket.x, middle_x, x_scale=bt_r_speed_x_scale, y_scale=max_speed*bt_r_speed_y_multiplier)
                if (len(processed_data.balls) != 0) and interesting_ball.distance > bt_ball_min_radius and interesting_ball.distance < bt_ball_max_radius:
                    speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=bt_x_speed_x_scale, y_scale=max_speed*bt_r_speed_y_multiplier)
                else:
                    speed_x = 0

                if not np.isnan(basket.distance):
                    robot.move(speed_x, bt_y_speed, speed_rot, int(basket.distance))
                else: 
                    robot.move(speed_x, bt_y_speed, speed_rot, 0)
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

                if xbox_cont.joystick_left_y > m_deadzone or xbox_cont.joystick_left_y < -m_deadzone:
                    joyY = xbox_cont.joystick_left_y

                if xbox_cont.joystick_left_x > m_deadzone or xbox_cont.joystick_left_x < -m_deadzone:
                    joyX = xbox_cont.joystick_left_x

                if xbox_cont.joystick_right_x > m_deadzone or xbox_cont.joystick_right_x < -m_deadzone:
                    joyRightX = -xbox_cont.joystick_right_x
                
                joyRTrig = xbox_cont.trigger_right

                #print(str(xboxcont.joystick_left_y) + " / " + str(xboxcont.joystick_left_x))

                speedy = joyY * m_y_scaling
                speedx = joyX * m_x_scaling
                speedr = joyRightX * m_r_scaling
                speedthrow = joyRTrig * m_thrower_scaling

                #print("Y: " + str(speedy) + " X: " + str(speedx) + " R: " + str(speedr))

                robot.move(speedx, speedy, speedr, speedthrow)
                
            else: # Unknown state
                # Considered as an error
                log.LOGSTATE("unknown state")
                log.LOGE("Exiting...")
                break
            # End of unknown

    except KeyboardInterrupt:
        print("\nExiting")
        cv2.destroyAllWindows()
        processor.stop()
        if referee_enabled:
            referee.close()
        robot.close()
        log.close()
        sys.exit()
