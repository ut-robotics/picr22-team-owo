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
    debug = True

    # Setup from our code
    robot = omni_motion.Omni_motion_robot()

    # Setup from example code
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0

    try:
        while(True):
            processedData = processor.process_frame(aligned_depth=False)

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

            # Movement logic
            speed_x = 0
            speed_y = 0
            speed_r = 0

            robot.move(speed_y, speed_x, speed_r)

            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\nExiting")
        sys.exit()
    
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        robot.close()
