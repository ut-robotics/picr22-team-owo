import segment
import numpy as np
import cPickle as pickle
import time
import cv2

with open('../colors.pkl', 'rb') as fh:
	colors_lookup = pickle.load(fh)
	
segment.set_table(colors_lookup)
		
cap = cv2.VideoCapture(0)
if not cap.isOpened():
	print('cam not opened')
	exit()
_, yuv = cap.read()
cap.release()

timea	= time.time()
for i in range(60):
	ob	= yuv.astype('uint32')
	fragmented	= colors_lookup[ob[:,:,0] + ob[:,:,1] * 0x100 + ob[:,:,2] * 0x10000]
	t_ball = (fragmented == 1).view('uint8')
	t_gatey = (fragmented == 2).view('uint8')
	t_gateb = (fragmented == 3).view('uint8')
print(time.time() - timea)
print(fragmented.shape)

while True:
	cv2.imshow('tava', t_ball * 255)
	if cv2.waitKey(1) & 0xff == ord('q'):
		break

mshape	= (480, 640)
segmented = np.zeros(mshape, dtype=np.uint8)
is_ball = np.zeros(mshape, dtype=np.uint8)
is_gatey = np.zeros(mshape, dtype=np.uint8)
is_gateb = np.zeros(mshape, dtype=np.uint8)

time.sleep(1)

timea	= time.time()
for i in range(60):
	segment.segment(yuv, segmented, is_ball, is_gatey, is_gateb)
print(time.time() - timea)

while True:
	cv2.imshow('tava', is_ball)
	if cv2.waitKey(1) & 0xff == ord('q'):
		break