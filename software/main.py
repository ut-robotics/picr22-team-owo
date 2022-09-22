#!/usr/bin/python3

# Python stuff
import time, sys

# Movement
import omni_motion

# Boot camp examples
sys.path.append("./picr22-boot-camp-programming")
import image_processor
import camera

# Open cv
import cv2

if __name__ == "__main__":
    print("Starting...")
    debug = False

    # Setup from our code
    robot = omni_motion.Omni_motion_robot()
    robot.start()

    # Setup from example code
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0

    middle_x = 424
    middle_y = 240

    state = None

    try:
        while(True):
            processedData = processor.process_frame(aligned_depth=True)

            frame_cnt +=1

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processedData.balls)))
                #if (frame_cnt > 1000):
                #    break

            # Debug frame (turn off when over ssh)
            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break


            # Main control logic
            if state == "wait":
                input()

            elif state == "ball_search":
                if len(processedData.balls) > 0:
                    state = "ball_move"
                else:
                    robot.move(0, 0, 3)
            
            elif state == "ball_move":
                if len(processedData.balls) > 0:
                    # Movement logic
                    speed_x = 0
                    speed_y = 0
                    speed_r = 0

                    #robot.move(speed_y, speed_x, speed_r)
                    interesting_ball = processedData.balls[-1]
                    print(interesting_ball)

                    if interesting_ball.x < 424 + 15 and interesting_ball.x > 424 - 15 and interesting_ball.distance <= 400:
                        state = "wait"
                    else:
                        if interesting_ball.x > 424 + 15 or interesting_ball.x < 424 - 15:
                            speed_x = (interesting_ball.x - middle_x) / 424 * 5
                            speed_r = -(interesting_ball.x - middle_x) / 424 * 10
                        if interesting_ball.distance > 400:
                            speed_y = (interesting_ball.distance - 400)*(15 - 0)/(2000 - 400)
                        print("x: %s, y: %s, r: %s" % (speed_x, speed_y, speed_r))
                        robot.move(speed_x, speed_y, speed_r)
                else:
                    state = "ball_search"

            elif state == "ball_throw":
                pass

            else:
                print("Unknown state")

    except KeyboardInterrupt:
        print("\nExiting")
        sys.exit()
    
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        robot.close()
