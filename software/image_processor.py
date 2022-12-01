#from asyncio.windows_events import NULL
import camera
import segment
import _pickle as pickle
import numpy as np
import cv2
import Color as c
import math
from numba import njit

# position calculation helper function
@njit
def calculatePosition(height, width, depth, origx, origy):
    x = int(origx + (width/2))
    y = int(origy + (height/2))
    if depth is None:
        dst = -242.0983 + (12373.93 - -242.0983)/(1 + math.pow((y/4.829652), 0.6903042))
    else:
        dst = depth[y, x]
    
    return x, y, dst

class Object():
    def __init__(self, x = -1, y = -1, size = -1, distance = -1, exists = False):
        self.x = x
        self.y = y
        self.size = size
        self.distance = distance
        self.exists = exists

    def __str__(self) -> str:
        return "[Object: x={}; y={}; size={}; distance={}; exists={}]".format(self.x, self.y, self.size, self.distance, self.exists)

    def __repr__(self) -> str:
        return "[Object: x={}; y={}; size={}; distance={}; exists={}]".format(self.x, self.y, self.size, self.distance, self.exists)


# results object of image processing. contains coordinates of objects and frame data used for these results
class ProcessedResults():

    def __init__(self, 
                balls=[], 
                basket_b = Object(exists = False), 
                basket_m = Object(exists = False), 
                color_frame = [],
                depth_frame = [],
                fragmented = [],
                debug_frame = []) -> None:


        self.balls = balls
        self.basket_b = basket_b
        self.basket_m = basket_m
        self.color_frame = color_frame
        self.depth_frame = depth_frame
        self.fragmented = fragmented

        # can be used to illustrate things in a separate frame buffer
        self.debug_frame = debug_frame


#Main processor class. processes segmented information
class ImageProcessor():
    def __init__(self, camera, color_config = "colors/colors.pkl", debug = False):
        self.camera = camera

        self.color_config = color_config
        with open(self.color_config, 'rb') as conf:
            self.colors_lookup = pickle.load(conf)
            self.set_segmentation_table(self.colors_lookup)

        self.fragmented	= np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

        self.t_balls = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.t_basket_b = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.t_basket_m = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

        self.debug = debug
        self.debug_frame = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

    def set_segmentation_table(self, table):
        segment.set_table(table)

    def start(self):
        self.camera.open()


    # will get lines from the image and return them as line equations
    def get_lines(self, image, fragmented):
        img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        # 30:400 proved to work
        gr_img = img[0:320]
        krn = 1 # kernel size for gauss
        #blur_img = cv2.GaussianBlur(gr_img, (krn, krn), 0)

        frag_sx, frag_sy = np.shape(fragmented)
        fragmented_white = np.zeros((frag_sx, frag_sy))
        fragmented_black = np.zeros((frag_sx, frag_sy))
        #print(f"tere hommikust: {frag_sx} {frag_sy}")

        #fragmentedblack[fragmentedblack == 6] = 0
        fragmented_black[fragmented == 6] = 1

        #fragmentedwhite[fragmentedwhite == 5] = 0
        fragmented_white[fragmented == 5] = 1
        
        
        open_kernel = np.ones((3,3), np.uint8)
        lowdilate_kernel = np.ones((3,3), np.uint8)

        detection_black = cv2.morphologyEx(fragmented_black, cv2.MORPH_CLOSE, open_kernel)

        fragmented_white = cv2.dilate(fragmented_white, lowdilate_kernel)
        fragmented_black = cv2.dilate(fragmented_black, lowdilate_kernel)

        fragmented_white = fragmented_white.astype(np.uint8) * 255
        fragmented_black = fragmented_black.astype(np.uint8) * 255
        detection_black = detection_black.astype(np.uint8) * 255

        # fragmented_white, fragmented_black, detection_black = process_line_images(fragmented)

        #low = 60
        #high = 100

        #ret, thresh = cv2.threshold(gr_img, low, high, cv2.THRESH_BINARY_INV)


        # different line detection parameters
        #scv2.imshow('hallo', thresh)
        low_thr = 50
        high_thr = 160
        edges = cv2.Canny(detection_black, low_thr, high_thr)
        rho = 1
        theta = np.pi / 180 * 1
        threshold = 30
        minline = 60
        maxgap = 30

        cropped = image[0:320]

        copy_img = np.copy(cropped) * 0

        # here happens the magic
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), minline, maxgap)
        
        lines_by_slope = []
        
        #print("lines: ", len(lines))
        if lines is None:
            return
        

        # calculates the line equations
        for line in lines:
            x1, y1, x2, y2 = line[0]
            midx = int((x1 + x2) / 2)
            midy = int((y1+y2)/2)

            if x1 == x2:
                continue
            slope = (y2 - y1) / (x2 - x1) # slope

            #antislope = 1/slope

            # LINE COLOUR SAMPLING
            sample_length = 10
            w_counter = 0
            b_counter = 0
            for i in range(sample_length):
                if (midy + i) >= 480 or (midy + i) < 0:
                    continue
                if (midx >= 640 or midx < 0):
                    continue
                if fragmented_white[midy + i, midx] != 0:
                    w_counter += 1
                if fragmented_black[midy - i, midx] != 0:
                    b_counter += 1
            if b_counter < 2 or w_counter < 2:
                b_counter = 0
                w_counter = 0
                continue
                    

            cv2.line(copy_img, (x1, y1), (x2, y2), (255, 0, 0), 1)


            intercept = y1 - (slope * x1) # intercept
            lines_by_slope.append((slope, intercept))

        if self.debug:
            #print("terre")
            #cv2.imshow("edges", edges)
            #cv2.imshow("lines", copy_img)
            #cv2.imshow("black", detection_black)
            #cv2.imshow("white", fragmentedwhite)
            #cv2.imshow("comb", comb_img)
            pass

        return lines_by_slope

    def stop(self):
        self.camera.close()

    # returns the balls from an already segmented image
    def analyze_balls(self, t_balls, fragments, depth, lines) -> list:

        krnl = np.ones((3,3), np.uint8)
        t_balls = cv2.dilate(t_balls, krnl)
        t_balls = cv2.erode(t_balls, krnl)

        contours, hierarchy = cv2.findContours(t_balls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        balls = []

        for contour in contours:

            size = cv2.contourArea(contour)

            if size < 20:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            ys	= np.array(np.arange(y + h, self.camera.rgb_height), dtype=np.uint16)
            xs	= np.array(np.linspace(x + w/2, self.camera.rgb_width / 2, num=len(ys)), dtype=np.uint16)

            obj_x, obj_y, obj_dst = calculatePosition(h, w, depth, x, y)
            
            above_line = False
            if lines is not None:
                for slope, interc in lines:
                    if obj_y < (slope * obj_x + interc + 0): # NB! 30/0 is the offset from line processing!
                        #print ("Ball " + str(obj_x) + "/" + str(obj_y) + "/" + str(obj_dst) + " is outside of the court")
                        # If the robot is not outside of this line (is on the court) and the ball is, this ball is discarded
                        above_line = True
                        break
                if above_line:
                    #print("ball at x: " + str(obj_x) + " y: " + str(obj_y) + "is above a line")
                    continue

            if self.debug:
                self.debug_frame[ys, xs] = [0, 0, 0]
                cv2.circle(self.debug_frame,(obj_x, obj_y), 10, (0,255,0), 2)
                #cv2.imshow("lines", copy_img)

            balls.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))

        balls.sort(key= lambda x: x.size)

        return balls

    # same thing for baskets
    def analyze_baskets(self, t_basket, depth, debug_color = (0, 255, 255)) -> list:

        krnl = np.ones((3,3), np.uint8)
        t_basket = cv2.morphologyEx(t_basket, cv2.MORPH_CLOSE, krnl)

        contours, hierarchy = cv2.findContours(t_basket, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        baskets = []
        for contour in contours:

            size = cv2.contourArea(contour)

            if size < 60:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            obj_x, obj_y, not_used = calculatePosition(h, w, depth, x, y)
            if depth is None:
                obj_dst = -242.0983 + (12373.93 - -242.0983)/(1 + math.pow((obj_y/4.829652), 0.6903042))
            else:
                obj_dst = np.average(depth[obj_y-5:obj_y+5, obj_x-5:obj_x+5])

            baskets.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))

        baskets.sort(key= lambda x: x.size)

        basket = next(iter(baskets), Object(exists = False))

        if self.debug:
            if basket.exists:
                cv2.circle(self.debug_frame,(basket.x, basket.y), 20, debug_color, -1)

        # Basket distance print for debug reasons
        #print("BASKET DISTANCE..... ", basket.distance)

        return basket

    def get_frame_data(self, aligned_depth = False):
        if self.camera.has_depth_capability():
            return self.camera.get_frames(aligned = aligned_depth)
        else:
            return self.camera.get_color_frame(), np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

    def process_frame(self, aligned_depth = False) -> ProcessedResults:
        color_frame, depth_frame = self.get_frame_data(aligned_depth = aligned_depth)

        segment.segment(color_frame, self.fragmented, self.t_balls, self.t_basket_m, self.t_basket_b)

        if self.debug:
            self.debug_frame = np.copy(color_frame)
        lines = self.get_lines(color_frame, self.fragmented)

        balls = self.analyze_balls(self.t_balls, self.fragmented, depth_frame, lines)
        basket_b = self.analyze_baskets(self.t_basket_b, depth_frame, debug_color=c.Color.BLUE.color.tolist())
        basket_m = self.analyze_baskets(self.t_basket_m, depth_frame, debug_color=c.Color.MAGENTA.color.tolist())

        return ProcessedResults(balls = balls, 
                                basket_b = basket_b, 
                                basket_m = basket_m, 
                                color_frame=color_frame, 
                                depth_frame=depth_frame, 
                                fragmented=self.fragmented, 
                                debug_frame=self.debug_frame)
