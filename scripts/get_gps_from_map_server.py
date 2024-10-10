#!/usr/bin/env python3

import rospy
import math 
import cv2
import os
import shlex
import pymap3d as pm
import numpy as np
from get_gps_from_map.srv import get_gps_from_map_srv
import threading
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point

mutex = threading.Lock()

####   https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Example:_Convert_a_GPS_coordinate_to_a_pixel_position_in_a_Web_Mercator_tile



### Variables for image tile processing
zoom = 18
dimension = 256


### Variables for image data
map_tile = np.zeros( (1*dimension, 1*dimension, 3), np.uint8)
map_img = np.zeros( (1*dimension, 1*dimension, 3), np.uint8)
ref_x_pixel = None
ref_y_pixel = None
x_tile = None
y_tile = None
selected_pixel_list = []
exit_loop = False


### Variables for reference GPS signal
reference_GPS = NavSatFix()


### Variables to store final transformed GPS to XYZ
tile_origin_GPS = NavSatFix()
tile_origin_xyz_wrt_reference_GPS = Point()
GPS_output = []
Transformed_points = []



def convert_GPS_to_pixels(latitude, longitude):

    x_epsg = longitude
    y_epsg = math.log(math.tan(math.radians(latitude)) + (1/math.cos(math.radians(latitude))))  # here log is natural log that is base e
    # print (f"x_epsg: {x_epsg}, y_epsg: {y_epsg}\n")

    x = 0.5 + x_epsg/360
    y = 0.5 - y_epsg/(2*math.pi)
    # print (f"x: {x}, y: {y}, zoom:{zoom}\n")

    N = 2**zoom
    x = N*x
    y = N*y
    
    x_tile = int(x)
    y_tile = int(y)
    # print (f"x_tile: {x_tile}, y_tile: {y_tile}\n")

    x_pixel = int(dimension*(x - x_tile))
    y_pixel = int(dimension*(y - y_tile))
    # print (f"x_pixel: {x_pixel}, y_pixel: {y_pixel}\n")

    resolution = 156543.03 * math.cos(math.radians(latitude)) / (N)

    img_url = "https://tile.openstreetmap.org/" + str(zoom) + "/" + str(x_tile) + "/" + str(y_tile) + ".png"

    return img_url, x_tile, y_tile, x_pixel, y_pixel, resolution
        

def convert_pixels_to_GPS(x_pixel, y_pixel, x_tile, y_tile):
    frac_x = 1.0*x_pixel/dimension
    frac_y = 1.0*y_pixel/dimension

    x = x_tile + frac_x
    y = y_tile + frac_y

    N = 2**zoom
    x = x/N
    y = y/N


    x_epsg = 360* (x - 0.5)
    y_epsg = 2*math.pi*(0.5 - y)
    

    longitude = x_epsg
    latitude = math.degrees(math.atan(math.sinh(y_epsg)))

    return latitude, longitude
    

def download_image(url, index=0):
    filename = f"/home/mohit/workspace_ws/src/get_gps_from_map/scripts/maps/map_{index}.png"
    if os.path.exists(filename):
        os.remove(filename)

    os.system(f"wget -c --output-document={filename} --read-timeout=5 --tries=0 {shlex.quote(url)}")
    img = cv2.imread(f"{filename}")
    return img


def click_event(event, x, y, flags, params):
    global selected_pixel_list, exit_loop
    if event == cv2.EVENT_LBUTTONDOWN:

        selected_data = (x,y)
        if len(selected_pixel_list) == 0:
            selected_pixel_list = [(x,y)]
        else:
            selected_pixel_list.append(selected_data)

    if event == cv2.EVENT_RBUTTONDOWN:
        exit_loop = True


def transform_points_from_GPS_to_XYZ(lat, lon, h, lat0, lon0, h0):
    x, y, z = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
    return x, y, z


def my_thread():
    global map_img, exit_loop
    cv2.namedWindow('tile_map', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('tile_map', click_event)
    while 1:
        cv2.imshow('tile_map', map_img)
        cv2.waitKey(50)
        # print ('exit_loop: ', exit_loop)



def handle_get_gps_from_map_srv(req):

    global map_tile, map_img, ref_x_pixel, ref_y_pixel, x_tile, y_tile, selected_pixel_list, exit_loop, \
           reference_GPS, tile_origin_GPS, tile_origin_xyz_wrt_reference_GPS, GPS_output, Transformed_points

    print (f"\n\n\n\n")
    print (f"request_to_update_reference_GPS {req.request_to_update_reference_GPS}\n \
           request_for_GPS: {req.request_for_GPS}\n\n")

    if req.request_to_update_reference_GPS:
        reference_GPS = req.reference_GPS

        ## Downloading the tile image corresponding to reference GPS signal
        if reference_GPS.latitude is not None:
            print ('Tile map Downloading')
            img_url, x_tile, y_tile, ref_x_pixel, ref_y_pixel, _ = convert_GPS_to_pixels(reference_GPS.latitude, reference_GPS.longitude)
            map_tile = download_image(img_url)
            exit_loop = False
            selected_pixel_list = []
            GPS_output = []
            Transformed_points = []

            ## Creating a copy of the tile to plot the selected pixels
            map_img = np.zeros( (1*dimension, 1*dimension, 3), np.uint8)
            map_img = map_tile.copy()

            ## Now Plotting the green dot at the reference GPS signal
            cv2.circle (map_img, (ref_x_pixel, ref_y_pixel), 3, (0,255,0), -1)

            ## Now calculating the tile map origin GPS cordinates which is at pixel(0,255), \
            ## and then calculate the relative distcance between tile map GPS and reference GPS for visulizing in RVIZ.
            tile_origin_GPS.latitude, tile_origin_GPS.longitude = convert_pixels_to_GPS(0, 255, x_tile, y_tile)
            tile_origin_xyz_wrt_reference_GPS.x, tile_origin_xyz_wrt_reference_GPS.y, tile_origin_xyz_wrt_reference_GPS.z = transform_points_from_GPS_to_XYZ(reference_GPS.latitude, reference_GPS.longitude, 0, tile_origin_GPS.latitude, tile_origin_GPS.longitude, 0)

           

    if req.request_for_GPS:

        while (1):

            ## Now Plotting the lines at selected pixels (single left click)
            for i in range(len(selected_pixel_list)):
                cv2.circle (map_img, (selected_pixel_list[i][0], selected_pixel_list[i][1]), 3, (255,0,0), -1)
                if i == 0:
                    cv2.line(map_img, (selected_pixel_list[i][0], selected_pixel_list[i][1]), (ref_x_pixel, ref_y_pixel), (0,0,0),1) 
                else:
                    cv2.line(map_img, (selected_pixel_list[i][0], selected_pixel_list[i][1]), (selected_pixel_list[i-1][0], selected_pixel_list[i-1][1]), (0,0,0),1)

           
                ## Transforming selected pixels to GPS cordinates
                GPS_for_selected_pixel = NavSatFix()
                GPS_for_selected_pixel.latitude, GPS_for_selected_pixel.longitude = convert_pixels_to_GPS(selected_pixel_list[i][0], selected_pixel_list[i][1], x_tile, y_tile)


                ## Transforming GPS cordinates to XYZ wrt to reference GPS signal
                Transformed_point = Point()
                Transformed_point.x, Transformed_point.y, Transformed_point.z = transform_points_from_GPS_to_XYZ(GPS_for_selected_pixel.latitude, GPS_for_selected_pixel.longitude, 0, reference_GPS.latitude, reference_GPS.longitude, 0)

                if i == 0:
                    GPS_output = [GPS_for_selected_pixel]
                    Transformed_points = [Transformed_point]
                else:
                    GPS_output.append(GPS_for_selected_pixel)
                    Transformed_points.append(Transformed_point)

            if exit_loop:
                print (f"len(selected_pixel_list): {len(selected_pixel_list)}")

                for i in range(len(selected_pixel_list)):
                    print (f"\n{i}, {Transformed_points[i].x}, {Transformed_points[i].y}, {Transformed_points[i].z} \
                           {GPS_output[i].latitude}, {GPS_output[i].longitude}")
                
                break
            


    print ('service_done')
    
    return tile_origin_GPS, tile_origin_xyz_wrt_reference_GPS, GPS_output, Transformed_points
        








def get_gps_from_map_srv_server():
    rospy.init_node('get_gps_from_map_srv_server')
    rospy.Service('get_gps_from_map_srv_server', get_gps_from_map_srv, handle_get_gps_from_map_srv)
    my_thread()
    rospy.spin()

if __name__ == '__main__':
    try:
        get_gps_from_map_srv_server()
    except rospy.ROSInterruptException: pass

