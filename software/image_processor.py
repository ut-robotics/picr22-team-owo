import camera
import segment
import _pickle as pickle
import numpy as np
import cv2
import Color as c


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

    def get_lines(self, image):
        img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        gr_img = img[30:400]
        krn = 5 # kernel size for gauss
        blur_img = cv2.GaussianBlur(gr_img, (krn, krn), 0)
        

        low = 58
        high = 150

        ret, thresh = cv2.threshold(blur_img, low, high, cv2.THRESH_BINARY_INV)

        #cv2.imshow('lines', thresh)
        low_thr = 50
        high_thr = 150
        edges = cv2.Canny(thresh, low_thr, high_thr)
        rho = 1
        theta = np.pi / 180
        threshold = 15
        minline = 50
        maxgap = 50

        cropped = image[30:400]

        copyimg = np.copy(cropped) * 0

        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), minline, maxgap)

        print("LINES..... ", lines)
        
        points = []
        #try:
        for line in lines:
            for x1, y1, x2, y2 in line:       
                points.append(((x1 + 0.0, y1 + 0.0), (x2 + 0.0, y2 + 0.0)))
                cv2.line(copyimg, (x1, y1), (x2, y2), (255, 0, 0), 5)
                lines_edges = cv2.addWeighted(cropped, 0.8, copyimg, 1, 0)
        #except:
        #    return

        cv2.imshow('lines', lines_edges)
        #return lines_edges

    def stop(self):
        self.camera.close()

    def analyze_balls(self, t_balls, fragments, depth) -> list:
        contours, hierarchy = cv2.findContours(t_balls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        balls = []

        for contour in contours:

            # ball filtering logic goes here. Example includes filtering by size and an example how to get pixels from
            # the bottom center of the fram to the ball

            size = cv2.contourArea(contour)


            # changed to 100 from 15, hopefully reducing errors in ball detection
            # needs testing
            #changed back to 15
            if size < 10:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            ys	= np.array(np.arange(y + h, self.camera.rgb_height), dtype=np.uint16)
            xs	= np.array(np.linspace(x + w/2, self.camera.rgb_width / 2, num=len(ys)), dtype=np.uint16)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            obj_dst = depth[obj_y, obj_x]

            if self.debug:
                self.debug_frame[ys, xs] = [0, 0, 0]
                cv2.circle(self.debug_frame,(obj_x, obj_y), 10, (0,255,0), 2)

            balls.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))

        balls.sort(key= lambda x: x.size)

        return balls

    def analyze_baskets(self, t_basket, debug_color = (0, 255, 255)) -> list:
        contours, hierarchy = cv2.findContours(t_basket, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        baskets = []
        for contour in contours:

            # basket filtering logic goes here. Example includes size filtering of the basket

            size = cv2.contourArea(contour)

            if size < 100:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            obj_dst = obj_y

            baskets.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))

        baskets.sort(key= lambda x: x.size)

        basket = next(iter(baskets), Object(exists = False))

        if self.debug:
            if basket.exists:
                cv2.circle(self.debug_frame,(basket.x, basket.y), 20, debug_color, -1)

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
            self.get_lines(color_frame)

        balls = self.analyze_balls(self.t_balls, self.fragmented, depth_frame)
        basket_b = self.analyze_baskets(self.t_basket_b, debug_color=c.Color.BLUE.color.tolist())
        basket_m = self.analyze_baskets(self.t_basket_m, debug_color=c.Color.MAGENTA.color.tolist())

        return ProcessedResults(balls = balls, 
                                basket_b = basket_b, 
                                basket_m = basket_m, 
                                color_frame=color_frame, 
                                depth_frame=depth_frame, 
                                fragmented=self.fragmented, 
                                debug_frame=self.debug_frame)

    
        


        
