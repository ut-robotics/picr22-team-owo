#from asyncio.windows_events import NULL
import camera
import segment
import _pickle as pickle
import numpy as np
import cv2
import Color as c
import math


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
        krn = 1 # kernel size for gauss
        blur_img = cv2.GaussianBlur(gr_img, (krn, krn), 0)
        

        low = 80
        high = 150

        ret, thresh = cv2.threshold(blur_img, low, high, cv2.THRESH_BINARY_INV)


        #scv2.imshow('hallo', thresh)
        low_thr = 50
        high_thr = 150
        edges = cv2.Canny(thresh, low_thr, high_thr)
        #edges = thresh
        rho = 1
        theta = np.pi / 180 * 1
        threshold = 50
        minline = 50
        maxgap = 40

        cropped = image[30:400]

        copyimg = np.copy(cropped) * 0

        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), minline, maxgap)

        #print("LINES..... ", len(lines))
        
        selected_lines = []
        selected_lines_list = []
        selected_lines_weights = []

        avglines = []

        linesbyslope = []
        
        points = []

        firstline = True

        #print("lines: ", len(lines))
        if lines is None:
            return
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # !!! ?????? points.append(((x1 + 0.0, y1 + 0.0), (x2 + 0.0, y2 + 0.0)))
            cv2.line(copyimg, (x1, y1), (x2, y2), (255, 0, 0), 1)
            
            if x1 == x2:
                continue
            slope = (y2 - y1) / (x2 - x1) # tous
            intercept = y1 - (slope * x1) # algordinaat
            linesbyslope.append((slope, intercept))
            '''
            length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2)) # joone pikkus

            if (firstline): # kui ei ole veel esimest joont hakatud tegutsema
                selected_lines.append(line)
                selected_lines_list.append([])
                selected_lines_list[0].append(line)
                selected_lines_weights.append([])
                selected_lines_weights[0].append(length)
                print("first line..")
                firstline = False

            else:
                found = False
                for ind, sel in enumerate(selected_lines):
                    #print(ind, " ", sel)
                    sx1, sy1, sx2, sy2 = sel[0]
                    sel_slope = (sy2 - sy1) / (sx2 - sx1) # tous
                    sel_intercept = sy1 - (slope * sx1) # algordinaat
                    sel_length = np.sqrt(((sy2 - sy1) ** 2) + ((sx2 - sx1) ** 2)) # joone pikkus

                    slope_tolerance = 0.5 # 0.1 on 10%
                    ordinaat_tolerance = 50 # pikslites

                    if (slope > (sel_slope * (1-slope_tolerance)) and slope < (sel_slope * (1+slope_tolerance))):
                    
                        if abs(intercept - sel_intercept) < ordinaat_tolerance:
                        
                            selected_lines_list[ind].append(line)
                            selected_lines_weights[ind].append(length)
                            #print("added line to group ", ind)
                            found = True
                            break
                            
                if not found:
                    selected_lines.append(line)
                    selected_lines_list.append([])
                    selected_lines_list[-1].append(line)
                    selected_lines_weights.append([])
                    selected_lines_weights[-1].append((length))

        for i in range(0, len(selected_lines)):
            lineamount = len(selected_lines_list[i])
            sumX1, sumY1, sumX2, sumY2 = (0, 0, 0, 0)
            for lin in selected_lines_list[i]:
                #sumX1, sumY1, sumX2, sumY2 += lin[0]
                sumX1 += lin[0][0]
                sumY1 += lin[0][1]
                sumX2 += lin[0][2]
                sumY2 += lin[0][3]
            avgX1 = sumX1 / lineamount
            avgY1 = sumY1 / lineamount
            avgX2 = sumX2 / lineamount
            avgY2 = sumY2 / lineamount

            avglines.append((int(avgX1), int(avgY1), int(avgX2), int(avgY2)))

        #for line in avglines:
        print("average lines: ", len(avglines))
        for x1, y1, x2, y2 in avglines:
            cv2.line(copyimg, (x1, y1), (x2, y2), (255, 0, 0), 5)
            
        '''

        #cv2.line(copyimg, (lines[0].x1, lines[0].y1), (lines[0].x2, lines[0].y2), (255, 0, 0), 5)

        lines_edges = cv2.addWeighted(cropped, 0.8, copyimg, 1, 0)

        #cv2.imshow('lines', lines_edges)
        
        #return lines_edges
        return linesbyslope

    def stop(self):
        self.camera.close()

    def analyze_balls(self, t_balls, fragments, depth, lines) -> list:
        contours, hierarchy = cv2.findContours(t_balls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        balls = []

        for contour in contours:

            # ball filtering logic goes here. Example includes filtering by size and an example how to get pixels from
            # the bottom center of the fram to the ball

            size = cv2.contourArea(contour)


            # changed to 100 from 15, hopefully reducing errors in ball detection
            # needs testing
            #changed back to 15
            if size < 14:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            ys	= np.array(np.arange(y + h, self.camera.rgb_height), dtype=np.uint16)
            xs	= np.array(np.linspace(x + w/2, self.camera.rgb_width / 2, num=len(ys)), dtype=np.uint16)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            if depth is 0:
                obj_dst = -242.0983 + (12373.93 - -242.0983)/(1 + math.pow((obj_y/4.829652), 0.6903042))
            else:
                obj_dst = depth[obj_y, obj_x]
            
            aboveline = False
            if lines is not None:
                for slope, interc in lines:
                    if obj_y < (slope * obj_x + interc + 30): # NB! 30 on joonte pildi lõikamise offset!
                        aboveline = True
                        break
                if aboveline:
                    continue

            if self.debug:
                self.debug_frame[ys, xs] = [0, 0, 0]
                cv2.circle(self.debug_frame,(obj_x, obj_y), 10, (0,255,0), 2)

            balls.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))

        balls.sort(key= lambda x: x.size)

        return balls

    def analyze_baskets(self, t_basket, depth, debug_color = (0, 255, 255)) -> list:
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
            if depth is 0:
                obj_dst = -242.0983 + (12373.93 - -242.0983)/(1 + math.pow((obj_y/4.829652), 0.6903042))
            else:
                #obj_dst = depth[obj_y, obj_x]
                obj_dst = np.average(depth[obj_y-2:obj_y+2, obj_x-2:obj_x+2])

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
        lines = self.get_lines(color_frame)

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

    
        


        
