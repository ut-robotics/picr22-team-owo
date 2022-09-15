import cv2
import numpy as np
import _pickle as pickle
import camera
import image_processor
from Color import *

def nothing(x):
    pass

cv2.namedWindow('image')
cv2.namedWindow('rgb')
cv2.namedWindow('mask')
cv2.moveWindow('mask', 400, 0)

try:
    with open('colors/colors.pkl', 'rb') as fh:
        colors_lookup = pickle.load(fh)
except:
    colors_lookup	= np.zeros(0x1000000, dtype=np.uint8)

#camera instance for normal web cameras
#cap = camera.OpenCVCamera(id = 2)
# camera instance for realsense cameras
cap = camera.RealsenseCamera(exposure = 100)

processor = image_processor.ImageProcessor(cap, debug=True)

cv2.createTrackbar('brush_size','image',3,10, nothing)
cv2.createTrackbar('noise','image',1,5, nothing)

mouse_x	= 0
mouse_y	= 0
brush_size	= 1
noise	= 1
p = 0

keyDict = {
    ord("g"): Color.GREEN,
    ord("m"): Color.MAGENTA,
    ord("b"): Color.BLUE,
    ord("f"): Color.ORANGE,
    ord("w"): Color.WHITE,
    ord("d"): Color.BLACK,
    ord("o"): Color.OTHER,
}

def change_color(noise, brush_size, mouse_x, mouse_y):
    ob	= rgb[
        max(0, mouse_y-brush_size):min(cap.rgb_height, mouse_y+brush_size+1),
        max(0, mouse_x-brush_size):min(cap.rgb_width, mouse_x+brush_size+1),:].reshape((-1,3)).astype('int32')
    noises		= range(-noise, noise+1)
    for r in noises:
        for g in noises:
            for b in noises:
                colors_lookup[((ob[:,0]+r) + (ob[:,1]+g) * 0x100 + (ob[:,2]+b) * 0x10000).clip(0,0xffffff)]	= p

# mouse callback function
def choose_color(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_x	= x
        mouse_y	= y
        brush_size	= cv2.getTrackbarPos('brush_size','image')
        noise		= cv2.getTrackbarPos('noise','image')
        change_color(noise, brush_size, mouse_x, mouse_y)

cv2.namedWindow('rgb')
cv2.setMouseCallback('rgb', choose_color)
cv2.setMouseCallback('mask', choose_color)

print("Quit: 'q', Save 's', Erase selected color 'e'")
print("Balls 'g', Magenta basket='m', Blue basket='b', Field='f', White='w', Black='d', Other='o'")

cap.open()

while(True):
    processedData = processor.process_frame()

    rgb = processedData.color_frame

    cv2.imshow('rgb', rgb)
    
    fragmented	= colors_lookup[rgb[:,:,0] + rgb[:,:,1] * 0x100 + rgb[:,:,2] * 0x10000]
    frame = np.zeros((cap.rgb_height, cap.rgb_width, 3), dtype=np.uint8)

    for color in Color:
        frame[fragmented == int(color)] = color.color

    cv2.imshow('mask', frame)
    
    k = cv2.waitKey(1) & 0xff

    if k == ord('q'):
        break
    elif k in keyDict:
        col = keyDict[k]
        print(col)
        p = int(col)
    elif k == ord('s'):
        with open('colors/colors.pkl', 'wb') as fh:
            pickle.dump(colors_lookup, fh, -1)
        print('saved')
    elif k == ord('e'):
        print('erased')
        colors_lookup[colors_lookup == p]	= 0

# When everything done, release the capture

cap.close()

cv2.destroyAllWindows()
