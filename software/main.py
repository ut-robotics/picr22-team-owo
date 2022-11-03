#!/usr/bin/python3

# Python
import time, sys, math
from enum import Enum
sys.path.append("./picr22-boot-camp-programming")

# Our code
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
    START = 3
    BALL_SEARCH = 4
    BALL_MOVE = 5
    BALL_ORBIT = 6
    BALL_THROW = 7
    # 8 and 9 are for thrower calibration
    INPUT = 8
    THROWER_CALIBRATION = 9


if __name__ == "__main__":
    # Housekeeping setup
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    log = Logging()
    log.LOGI("Starting...")

    # Mainboard stuff setup
    max_speed = 10
    max_speed_inner = 15
    ball_good_range = 300
    robot = mainboard.Mainboard(max_speed_inner, log)
    robot.start()

    # Control logic setup
    debug = False
    state = State.WAIT # Initial state
    thrower_speed = 0
    calib_first_time = True
    calibration_data = []
    basketColor = TargetBasket.MAGENTA # currently defaults to magenta for testing purposes
    thrower_time_start = 0

    # Vision setup
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()
    middle_x = cam.rgb_width / 2
    middle_y = cam.rgb_height / 2

    # Referee commands
    referee = ref_cmd.Referee_cmd_client(log)
    ref_first_start = True
    robot_name = "OWO"
    referee.start()

    try:
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
            msg = referee.get_cmd()
            if msg is not None:
                if robot.name in msg["targets"]:
                    if msg["signal"] == "start":
                        log.LOGI("Start signal received")
                        # ref_first_start used if we want to differentiate between start of the match and resuming from a stop
                        if ref_first_start:
                            log.LOGI("Match started!")
                            ref_first_start = False
                            state = State.START
                            if msg["baskets"][msg["targets"].find("OWO")] == 'blue':
                                basket_color = TargetBasket.BLUE
                            elif msg["baskets"][msg["targets"].find("OWO")] == 'magenta':
                                basket_color = TargetBasket.MAGENTA
                            else:
                                log.LOGE("Basket color error")
                        else:
                            state = State.BALL_SEARCH
                    elif msg["signal"] == "stop":
                        log.LOGI("Paused signal received")
                        state = State.PAUSED
            # End of referee commands

            # Main control logic uses a state machine
            if state == State.WAIT:
                log.LOGSTATE("waiting")
                input() # For making a break
                state = State.BALL_SEARCH
                continue
            # End of wait

            # States used for thrower calibration
            elif state == State.INPUT:
                thrower_speed = int(input("Speed:")) # Input speed for thrower
                state = State.THROWER_CALIBRATION

            elif state == State.THROWER_CALIBRATION: # Spin the motor for 10 seconds with the speed given in state "input"
                if calib_first_time:
                    calib_first_time = False
                    start_time = time.perf_counter()

                if (time.perf_counter() - start_time) > 10:
                    print("Average:", np.average(calibration_data))
                    print("Speed:", thrower_speed)
                    state = State.BALL_INPUT
                    calib_first_time = True
                    calibration_data = []
                    continue

                if (processedData.basket_m.exists):
                    print("Distance:", processedData.basket_m.distance)
                    calibration_data.append(processedData.basket_m.distance)
                    robot.throw(thrower_speed)
                else: 
                    print("No basket")
                    continue
            # End of states used for thrower calibration

            elif state == State.START:
                state = State.BALL_SEARCH
                continue

            elif state == State.BALL_SEARCH:
                log.LOGSTATE("ball_search")
                if len(processedData.balls) > 0:
                    state = State.BALL_MOVE
                    continue
                else:
                    robot.move(0, 0, 6)
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
                            speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=2000, y_scale=max_speed)
                            speed_r = -sigmoid_controller(interesting_ball.x, middle_x, x_scale=1200, y_scale=max_speed)
                        if interesting_ball.distance > ball_good_range:
                            speed_y = sigmoid_controller(interesting_ball.distance, ball_good_range, x_scale=1400, y_scale=max_speed)
                        print(f"x: {speed_x}, y: {speed_y}, r: {speed_r}, dist: {interesting_ball.distance}, b.x: {interesting_ball.x}, b.y: {interesting_ball.y}")
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
                        state = State.WAIT
                        continue

                    # Determining the correct basket
                    if basketColor == TargetBasket.MAGENTA:
                        basket = processedData.basket_m
                    elif basketColor == TargetBasket.BLUE:
                        basket = processedData.basket_b
                    else:
                        log.LOGE("Basket color invalid")

                    if basket.exists:
                        print("Basket x:", basket.x, "/", middle_x)
                        #if (processedData.basket_m.x > (middle_x + 1) or processedData.basket_m.x < (middle_x - 1)):
                        basket_tolerance = 13
                        if abs(basket.x - middle_x) < basket_tolerance:
                            state = State.BALL_THROW
                            thrower_time_start = time.perf_counter()
                            continue
                        
                        speed_x = -sigmoid_controller(basket.x, middle_x, x_scale=1500, y_scale=(max_speed - 2))
                        #print("rotational speed:", rot)

                        robot.orbit(400, speed_x, interesting_ball.distance, interesting_ball.x)

                    else:
                        robot.orbit(400, 2, interesting_ball.distance, interesting_ball.x)
                else:
                    print("no ball........")
                    state = State.BALL_SEARCH
                    continue
            # End of ball_orbit

            elif state == State.BALL_THROW:
                log.LOGSTATE("ball_throw")

                if basketColor == TargetBasket.MAGENTA:
                    basket = processedData.basket_m
                elif basketColor == TargetBasket.BLUE:
                    basket = processedData.basket_b
                else:
                    log.LOGE("Basket color invalid")

                if len(processedData.balls) > 0:
                    interesting_ball = processedData.balls[-1]

                if (thrower_time_start + 3 < time.perf_counter()):
                    state = State.BALL_SEARCH


                speed_rot = -sigmoid_controller(basket.x, middle_x, x_scale=900, y_scale=(max_speed / 2))
                if (len(processedData.balls) != 0) and interesting_ball.distance > 400 and interesting_ball.distance < 600:
                    speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=2200, y_scale=max_speed / 3)
                else:
                    speed_x = 0

                speed_y = 0.7

                if (basket.distance != math.nan):
                    robot.move(speed_x, speed_y, speed_rot, int(basket.distance))
                else: 
                    robot.move(speed_x, speed_y, speed_rot, 0)
                continue
            # End of ball_throw

            else: # Unknown state
                # Considered as an error
                log.LOGSTATE("unknown state")
                log.LOGE("Exiting...")
                break
            # End of unknown

    except KeyboardInterrupt:
        print("\nExiting")
        sys.exit()
    
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        robot.close()
