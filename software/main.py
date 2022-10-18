#!/usr/bin/python3

import time, sys

import omni_motion
from robot_utilities import *

# Boot camp examples
sys.path.append("./picr22-boot-camp-programming")
import image_processor
import camera

# Open cv
import cv2

if __name__ == "__main__":
    print("Starting...")
    init_logging()
    debug = False

    # Setup from our code
    max_speed = 10
    max_speed_inner = 15
    ball_good_range = 375
    robot = omni_motion.Omni_motion_robot(max_speed_inner)
    robot.start()

    # Setup from example code
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()

    # Constants etc.
    # Housekeeping
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    # Camera constants
    middle_x = 424
    middle_y = 240
    # Control logic constants
    state = sys.argv[1] # Initial state
    using_magenta = True # If throwing into magenta basket set to true, if throwing into blue, set to false
    thrower_speed = 0
    first_time = True
    calibration_data = []


    thrower_time_start = 0
    basket_ok_counter = 0

    try:
        while(True):
            # Getting camera data
            processedData = processor.process_frame(aligned_depth=True)

            # Housekeeping stuff
            frame_cnt +=1
            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                LOGI("FPS: {}, framecount: {}".format(fps, frame_cnt))
                LOGI("ball_count: {}".format(len(processedData.balls)))
            # End of housekeeping stuff

            # Debug frame (turn off when over ssh)
            if debug:
                debug_frame = processedData.debug_frame
                cv2.imshow('debug', debug_frame)
                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
            # End of housekeeping

            # Main control logic uses a state machine
            if state == "wait":
                LOGSTATE("waiting")
                input() # For making a break
                state = "ball_search"
                continue
            # End of wait

            # States used for thrower calibration
            elif state == "input":
                thrower_speed = int(input("Speed:")) # Input speed for thrower
                state = "calibration"

            elif state == "calibration": # Spin the motor for 10 seconds with the speed given in state "input"
                if first_time:
                    first_time = False
                    start_time = time.perf_counter()

                if (time.perf_counter() - start_time) > 10:
                    print("Average:", np.average(calibration_data))
                    print("Speed:", thrower_speed)
                    state = "input"
                    first_time = True
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

            elif state == "ball_search":
                LOGSTATE("ball_search")
                if len(processedData.balls) > 0:
                    state = "ball_move"
                    continue
                else:
                    robot.move(0, 0, 6)
            # End of ball_search
            
            # Moving towards ball
            elif state == "ball_move":
                LOGSTATE("ball_move")
                if len(processedData.balls) > 0:
                    # Movement logic
                    speed_x = 0
                    speed_y = 0
                    speed_r = 0

                    # Choosing the closest ball
                    interesting_ball = processedData.balls[-1]
                    #print("Ball:", interesting_ball)

                    if interesting_ball.x < middle_x + 25 and interesting_ball.x > middle_x - 25 and interesting_ball.distance <= 475:
                        state = "ball_orbit"
                        basket_ok_counter = 0
                        continue
                    else:
                        if interesting_ball.x > middle_x + 2 or interesting_ball.x < middle_x - 2:
                            speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=2000, y_scale=max_speed)
                            speed_r = -sigmoid_controller(interesting_ball.x, middle_x, x_scale=1200, y_scale=max_speed)
                        if interesting_ball.distance > ball_good_range:
                            speed_y = sigmoid_controller(interesting_ball.distance, ball_good_range, x_scale=1500, y_scale=max_speed)
                        print(f"x: {speed_x}, y: {speed_y}, r: {speed_r}, dist: {interesting_ball.distance}, b.x: {interesting_ball.x}, b.y: {interesting_ball.y}")
                        robot.move(speed_x, speed_y, speed_r)
                else:
                    state = "ball_search"
                    continue
            # End of ball_move

            # Orbiting around ball until correct basket is found
            elif state == "ball_orbit":
                LOGSTATE("ball_orbit")
                

                if len(processedData.balls) > 0:
                    interesting_ball = processedData.balls[-1]

                    # For checking if the ball is still in position
                    if interesting_ball.distance > 550:
                        LOGE("Invalid radius, radius: " + str(interesting_ball.distance))
                        state = "wait"
                        continue

                    # Determining the correct basket
                    if using_magenta:
                        basket = processedData.basket_m
                    else:
                        basket = processedData.basket_b

                    if basket.exists:
                        print("Basket x:", basket.x, "/", middle_x)
                        #if (processedData.basket_m.x > (middle_x + 1) or processedData.basket_m.x < (middle_x - 1)):
                        if (basket.distance < 1300):
                            basket_tolerance = 15
                        else:
                            basket_tolerance = 10

                        if abs(basket.x - middle_x) < basket_tolerance:
                            print("basket ok counter...", basket_ok_counter)
                            if (basket_ok_counter > 4):
                                state = "ball_throw"
                                thrower_time_start = time.perf_counter()
                                continue
                            else:
                                basket_ok_counter += 1
                            
                        else:
                            basket_ok_counter = 0
                        
                        speed_x = -sigmoid_controller(basket.x, middle_x, x_scale=1500, y_scale=(max_speed - 2))
                        #print("rotational speed:", rot)

                        robot.orbit(400, speed_x, interesting_ball.distance, interesting_ball.x)

                    else:
                        robot.orbit(400, 2, interesting_ball.distance, interesting_ball.x)
                else:
                    print("no ball........")
                    state == "ball_search"
                    continue
            # End of ball_orbit

            elif state == "ball_throw":
                LOGSTATE("ball_throw")

                if using_magenta:
                    basket = processedData.basket_m
                else:
                    basket = processedData.basket_b

                if len(processedData.balls) > 0:
                    interesting_ball = processedData.balls[-1]

                if (thrower_time_start + 3 < time.perf_counter()):
                    state = "ball_search"


                speed_rot = -sigmoid_controller(basket.x, middle_x, x_scale=2000, y_scale=(max_speed))
                if (len(processedData.balls) != 0) and interesting_ball.distance > 400 and interesting_ball.distance < 600:
                    speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=2200, y_scale=max_speed / 3)
                else:
                    speed_x = 0

                speed_y = 0.7

                throw_speed = basket.distance*0.3166512 + 471.4378
                robot.move(speed_x, speed_y, speed_rot, int(throw_speed))
                continue
            # End of ball_throw

            else: # Unknown state
                # Considered as an error
                LOGSTATE("unknown state")
                LOGE("Exiting...")
                break
            # End of unknown

    except KeyboardInterrupt:
        print("\nExiting")
        sys.exit()
    
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        robot.close()