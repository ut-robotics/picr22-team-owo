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
    ball_good_range = 370
    robot = omni_motion.Omni_motion_robot()
    robot.start(max_speed)

    # Setup from example code
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()

    # General constants etc.
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0

    middle_x = 424
    middle_y = 240

    state = "wait" # Initial state
    using_magenta = True # If throwing into magenta basket set to true, if throwing into blue, set to false

    try:
        while(True):
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

            # Debug frame (turn off when over ssh)
            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
            # End of housekeeping

            # Main control logic using a state machine
            if state == "wait":
                LOGSTATE("waiting")
                input() # For making a break
                state = "ball_search"
            # End of wait

            elif state == "ball_search":
                LOGSTATE("ball_search")
                if len(processedData.balls) > 0:
                    state = "ball_move"
                else:
                    robot.move(0, 0, 5)
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

                    if interesting_ball.x < middle_x + 5 and interesting_ball.x > middle_x - 5 and interesting_ball.distance <= 400:
                        state = "ball_orbit"
                    else:
                        if interesting_ball.x > middle_x + 1 or interesting_ball.x < middle_x - 1:
                            speed_x = sigmoid_controller(interesting_ball.x, middle_x, x_scale=max_speed/10, y_scale=100)
                            speed_r = sigmoid_controller(interesting_ball.x, middle_x, x_scale=max_speed/10, y_scale=-100)
                        if interesting_ball.distance > 370:
                            speed_y = sigmoid_controller(interesting_ball.distance, ball_good_range, x_scale=max_speed/10, y_scale=1000)
                        print("x: %s, y: %s, r: %s" % (speed_x, speed_y, speed_r))
                        robot.move(speed_x, speed_y, speed_r)
                else:
                    state = "ball_search"
            # End of ball_move

            # Orbiting around ball until correct basket is found
            elif state == "ball_orbit":
                LOGSTATE("ball_orbit")

                if len(processedData.balls) > 0:
                    interesting_ball = processedData.balls[-1]

                    # For checking if the ball is still in position
                    if interesting_ball.distance > 450:
                        LOGE("Invalid radius")
                        state = "wait"
                        continue

                    # Determining the correct basket
                    if using_magenta:
                        basket = processedData.basket_m
                    else:
                        basket = processedData.basket_b

                    if basket.exists:
                        if (processedData.basket_m.x > middle_x + 1 or processedData.basket_m.x < middle_x - 1):
                            state = "ball_throw"
                            continue
                        
                        rot = sigmoid_controller(interesting_ball.x, middle_x, x_scale=max_speed/10, y_scale=-100)
                        print("rotational speed:", rot)
                        robot.orbit(400, rot/10, interesting_ball.distance, interesting_ball.x)
                    else:
                        robot.orbit(400, 2, interesting_ball.distance, interesting_ball.x)
                else:
                    state == "ball_search"
            # End of ball_orbit

            elif state == "ball_throw":
                LOGSTATE("ball_throw")
                state = "wait"
            # End of ball_throw

            else:
                LOGSTATE("unknown state")
                LOGE("Exiting...")
                break
                # Considered as an error
            # End of unknown

    except KeyboardInterrupt:
        print("\nExiting")
        sys.exit()
    
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        robot.close()
