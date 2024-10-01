import cv2
import numpy as np


def click_event(event, x, y, flags, params):
    global selected_pixel_list, exit_loop
    if event == cv2.EVENT_LBUTTONDOWN:

        selected_data = (x,y)
        if len(selected_pixel_list) == 0:
            selected_pixel_list = [(x,y)]
        else:
            selected_pixel_list.append(selected_data)

    if event == cv2.EVENT_RBUTTONDOWN:
        selected_pixel_list = []
        exit_loop = True

cc = 1
img_data = None
# cv2.namedWindow('tile_map', cv2.WINDOW_NORMAL) 
# cv2.setMouseCallback('tile_map', click_event)
while (cc < 20):

   

   img_data = 255*np.ones( (256, 256, 3), np.uint8)
   if cc % 2 == 0:
    img_data = 0*np.ones( (256, 256, 3), np.uint8)
   cv2.imshow('tile_map', img_data)
   cv2.waitKey(0)
   # cv2.destroyAllWindows()

   cc += 1
