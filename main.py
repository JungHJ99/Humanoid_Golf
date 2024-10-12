# -*- coding: utf-8 -*-
# main_241012_pbs_01.py
# 구현 : 폴대 2cm로 두꺼워도 홀 인식 원활하게

import platform
import numpy as np
import argparse
import cv2
import serial
import time
import sys
from threading import Thread
import csv
import math

from picamera import PiCamera
from picamera.array import PiRGBArray

X_255_point = 0
Y_255_point = 0
X_Size = 0
Y_Size = 0
Area = 0
Angle = 0
#-----------------------------------------------
Top_name = 'mini CTS5 setting'
hsv_Lower = 0
hsv_Upper = 0

hsv_Lower0 = 0
hsv_Upper0 = 0

hsv_Lower1 = 0
hsv_Upper1 = 0

#----------- 
color_num = [   0,  1,  2,  3,  4]
    
h_max =     [ 179,240, 140,100,110]
h_min =     [  86,0,  0, 30, 74]
    
s_max =     [ 121,70,130,140,255]
s_min =     [ 100, 0,85, 100,133]
    
v_max =     [ 255,175,180,100,255]
v_min =     [ 180, 0,100, 60,104]
    
min_area =  [  3, 30, 50, 10, 10]

now_color = 0
serial_use = 1
serial_port =  None
Temp_count = 0
Read_RX =  0

mx,my = 0,0

threading_Time = 5/1000.

Config_File_Name ='Cts5_v1.dat'
    
#-----------------------------------------------

def nothing(x):
    pass

#-----------------------------------------------
def create_blank(width, height, rgb_color=(0, 0, 0)):

    image = np.zeros((height, width, 3), np.uint8)
    color = tuple(reversed(rgb_color))
    image[:] = color

    return image
#-----------------------------------------------
def draw_str2(dst, target, s):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 255), lineType=cv2.LINE_AA)
#-----------------------------------------------
def draw_str3(dst, target, s):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), lineType=cv2.LINE_AA)
#-----------------------------------------------
def draw_str_height(dst, target, s, height):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, height, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, height, (255, 255, 255), lineType=cv2.LINE_AA)
#-----------------------------------------------
def clock():
    return cv2.getTickCount() / cv2.getTickFrequency()
#-----------------------------------------------

def Trackbar_change(now_color):
    global  hsv_Lower,  hsv_Upper
    hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
    hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])

#-----------------------------------------------
def Hmax_change(a):
    
    h_max[now_color] = cv2.getTrackbarPos('Hmax', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Hmin_change(a):
    
    h_min[now_color] = cv2.getTrackbarPos('Hmin', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Smax_change(a):
    
    s_max[now_color] = cv2.getTrackbarPos('Smax', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Smin_change(a):
    
    s_min[now_color] = cv2.getTrackbarPos('Smin', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Vmax_change(a):
    
    v_max[now_color] = cv2.getTrackbarPos('Vmax', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Vmin_change(a):
    
    v_min[now_color] = cv2.getTrackbarPos('Vmin', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def min_area_change(a):
   
    min_area[now_color] = cv2.getTrackbarPos('Min_Area', Top_name)
    if min_area[now_color] == 0:
        min_area[now_color] = 1
        cv2.setTrackbarPos('Min_Area', Top_name, min_area[now_color])
    Trackbar_change(now_color)
#-----------------------------------------------
def Color_num_change(a):
    global now_color, hsv_Lower,  hsv_Upper
    now_color = cv2.getTrackbarPos('Color_num', Top_name)
    cv2.setTrackbarPos('Hmax', Top_name, h_max[now_color])
    cv2.setTrackbarPos('Hmin', Top_name, h_min[now_color])
    cv2.setTrackbarPos('Smax', Top_name, s_max[now_color])
    cv2.setTrackbarPos('Smin', Top_name, s_min[now_color])
    cv2.setTrackbarPos('Vmax', Top_name, v_max[now_color])
    cv2.setTrackbarPos('Vmin', Top_name, v_min[now_color])
    cv2.setTrackbarPos('Min_Area', Top_name, min_area[now_color])

    hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
    hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])
#----------------------------------------------- 
def TX_data(ser, one_byte):  # one_byte= 0~255
    time.sleep(0.1)
    #ser.write(chr(int(one_byte)))          #python2.7
    ser.write(serial.to_bytes([one_byte]))  #python3

#-----------------------------------------------
def RX_data(serial):
    global Temp_count
    try:
        if serial.inWaiting() > 0:
            result = serial.read(1)
            RX = ord(result)
            return RX
        else:
            return 0
    except:
        Temp_count = Temp_count  + 1
        print("Serial Not Open " + str(Temp_count))
        return 0
        pass
#-----------------------------------------------

#*************************
# mouse callback function
def mouse_move(event,x,y,flags,param):
    global mx, my

    if event == cv2.EVENT_MOUSEMOVE:
        mx, my = x, y


# *************************
def RX_Receiving(ser):
    global receiving_exit,threading_Time

    global X_255_point
    global Y_255_point
    global X_Size
    global Y_Size
    global Area, Angle


    receiving_exit = 1
    while True:
        if receiving_exit == 0:
            break
        time.sleep(threading_Time)
        
        while ser.inWaiting() > 0:
            result = ser.read(1)
            RX = ord(result)
            print ("RX=" + str(RX))
            

def GetLengthTwoPoints(XY_Point1, XY_Point2):
    return math.sqrt( (XY_Point2[0] - XY_Point1[0])**2 + (XY_Point2[1] - XY_Point1[1])**2 )
# *************************
def FYtand(dec_val_v ,dec_val_h):
    return ( math.atan2(dec_val_v, dec_val_y) * (180.0 / math.pi))
# *************************
#degree 값을 라디안 값으로 변환하는 함수
def FYrtd(rad_val ):
    return  (rad_val * (180.0 / math.pi))

# *************************
# 라디안값을 degree 값으로 변환하는 함수
def FYdtr(dec_val):
    return  (dec_val / 180.0 * math.pi)

# *************************
def GetAngleTwoPoints(XY_Point1, XY_Point2):
    xDiff = XY_Point2[0] - XY_Point1[0]
    yDiff = XY_Point2[1] - XY_Point1[1]
    cal = math.degrees(math.atan2(yDiff, xDiff)) + 90
    if cal > 90:
        cal =  cal - 180
    return  cal
# *************************


    

#************************
def hsv_setting_save():

    global Config_File_Name, color_num
    global color_num, h_max, h_min 
    global s_max, s_min, v_max, v_min, min_area
    
    try:
    #if 1:
        saveFile = open(Config_File_Name, 'w')
        i = 0
        color_cnt = len(color_num)
        while i < color_cnt:
            text = str(color_num[i]) + ","
            text = text + str(h_max[i]) + "," + str(h_min[i]) + ","
            text = text + str(s_max[i]) + "," + str(s_min[i]) + ","
            text = text + str(v_max[i]) + "," + str(v_min[i]) + ","
            text = text + str(min_area[i])  + "\n"
            saveFile.writelines(text)
            i = i + 1
        saveFile.close()
        print("hsv_setting_save OK")
        return 1
    except:
        print("hsv_setting_save Error~")
        return 0
    

    
#************************
def hsv_setting_read():
    global Config_File_Name
    global color_num, h_max, h_min 
    global s_max, s_min, v_max, v_min, min_area

    #try:
    if 1:
        with open(Config_File_Name) as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            i = 0
            
            for row in readCSV:
                color_num[i] = int(row[0])
                h_max[i] = int(row[1])
                h_min[i] = int(row[2])
                s_max[i] = int(row[3])
                s_min[i] = int(row[4])
                v_max[i] = int(row[5])
                v_min[i] = int(row[6])
                min_area[i] = int(row[7])
                
                i = i + 1
              
        csvfile.close()
        print("hsv_setting_read OK")
        return 1
    #except:
    #    print("hsv_setting_read Error~")
    #    return 0

def ball_detecting(mask):

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # initialize parameter

    X_Size = 0
    Y_Size = 0
    X_255_point = 0
    Y_255_point = 0
    cx_ball = 0
    cy_ball = 0
    ball_detected = False
    Area = 0
    Angle = 0

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((X, Y), radius) = cv2.minEnclosingCircle(c)

        Area = cv2.contourArea(c) / min_area[0]

        if Area > 255:
            Area = 255

        if Area > min_area[0]:
            x4, y4, w4, h4 = cv2.boundingRect(c)
            cv2.rectangle(frame, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 0), 2)
            
            X_Size = int((255.0 / W_View_size) * w4)
            Y_Size = int((255.0 / H_View_size) * h4)
            X_255_point = int((255.0 / W_View_size) * X)
            Y_255_point = int((255.0 / H_View_size) * Y)
            cx_ball = x4 + w4 / 2
            cy_ball = y4 + h4 / 2
            ball_detected = True

        else:
            ball_detected = False
            
    else:
        X_255_point = 0
        Y_255_point = 0
        X_Size = 0
        Y_Size = 0
        Area = 0
        Angle = 0

    return X_Size, Y_Size, X_255_point, Y_255_point, cx_ball, cy_ball, ball_detected, Area, Angle

# parameter for hole_detecting()

min_area_hole = 0 # 5000
max_area_hole = 500000
min_circularity_hole = 0 # 0.7
max_aspect_ratio_hole = 10 # 1.5

def hole_detecting(frame, mask, hsv, min_area, max_area, min_circularity, max_aspect_ratio):

    # GaussianBlur
    blurred_image = cv2.GaussianBlur(mask, (5, 5), 0)

    # Morph Close
    kernel = np.ones((20, 20), np.uint8)    # kernel = np.ones((10, 10), np.uint8)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # kenel 크기의 작은 구멍을 메움 / 2cm 폴대 무시하도록 kernel 키움


    # 일정 크기 이상인 노란색 면적의 윤곽선 반환
    contours, _ = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    hole_detected = False
    largest_ellipse = None
    largest_contour = None
    largest_area = 0
    cX, cY, cR = 0, 0, 0
    x1, x2, y1, y2 = 0, 0, 0, 0
    largest_cX, largest_cY, largest_cR = 0, 0, 0
    largest_x1, largest_x2, largest_y1, largest_y2, largest_h_mean, largest_s_mean, largest_v_mean = 0, 0, 0, 0, 0, 0, 0
    
    W_View_size =  800  #320  #640
    #H_View_size = int(W_View_size / 1.777)
    H_View_size = int(W_View_size / 1.333)

    # 필터링을 위한 파라미터 계산
    for cnt in contours:
        
        if len(cnt) >= 5:
            ellipse = cv2.fitEllipse(cnt)
            center, axes, angle = ellipse
            major_axis = max(axes)
            minor_axis = min(axes)

            if minor_axis >0:
                aspect_ratio = major_axis / minor_axis
            else:
                continue

            contour_area = cv2.contourArea(cnt)
            arc_length = cv2.arcLength(cnt, True)
            circularity = 4 * np.pi * (contour_area / (arc_length ** 2))
            
            try:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cR = int(round(math.sqrt(0.1 * contour_area)))

                y1 = cY - cR
                y2 = cY + cR
                x1 = cX - cR
                x2 = cX + cR
            
                center_region = hsv[y1:y2, x1:x2]

            except:     #240921 error
                print([y1, y2, x1, x2, cX, cY, cR])
                raise 
                

            if center_region.size == 0:
                print("center_region.size == 0")
                continue
            
            center_region_h, center_region_s, center_region_v = cv2.split(center_region)
            h_mean = np.mean(center_region_h)
            s_mean = np.mean(center_region_s)
            v_mean = np.mean(center_region_v)

            # 필터링 조건
            if (contour_area >= min_area and            # 윤곽선의 면적이 최소 면적 이상
                contour_area <= max_area and            # 윤곽선의 면적이 최대 면적 이하
                circularity >= min_circularity and      # 윤곽선의 원형도가 최소 원형도 이상
                aspect_ratio <= max_aspect_ratio and    # 윤곽선의 종횡비가 최대 종횡비 이하
                h_min[2] <= h_mean <= h_max[2] and      # 윤곽선 중심의 hue 값이 검정색 범위 이내 / frame 기준이므로 closing 영향 X
                s_min[2] <= s_mean <= s_max[2] and      # 윤곽선 중심의 saturation 값이 검정색 범위 이내 / frame 기준이므로 closing 영향 X
                v_min[2] <= v_mean <= v_max[2]):        # 윤곽선 중심의 value 값이 검정색 범위 이내 / frame 기준이므로 closing 영향 X

                # 가장 큰 원(=홀) 찾기
                if contour_area > largest_area:
                    largest_area = contour_area
                    largest_ellipse = ellipse
                    largest_contour = cnt
                    hole_detected = True
                    largest_cX = cX
                    largest_cY = cY
                    largest_cR = cR

                    largest_x1 = x1
                    largest_x2 = x2
                    largest_y1 = y1
                    largest_y2 = y2
                    largest_h_mean = h_mean
                    largest_s_mean = s_mean
                    largest_v_mean = v_mean

    # print("x1: {}, x2: {}, y1: {}, y2: {}".format(largest_x1, largest_x2, largest_y1, largest_y2))

    # print("h_mean: {}".format(largest_h_mean))
    # print("s_mean: {}".format(largest_s_mean))
    # print("v_mean: {}".format(largest_v_mean))

    # 홀의 윤곽선 표시, 중심 좌표 계산
    if largest_ellipse is not None:
        cv2.drawContours(frame, [largest_contour], -1, (255, 0, 0), 2)

    # 홀의 면적, 중심 좌표 반환
    return hole_detected, largest_area, (largest_cX, largest_cY), closing

def border_before_hole_detecting(frame, mask, cx_hole, cy_hole, w_view_size, h_view_size, area_threshold, safety_thickness):

    border_before_hole_detected = False

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    start_x, start_y = (w_view_size // 2, h_view_size)
    end_x, end_y = (cx_hole, cy_hole)

    line_points = get_line_points_with_thickness(start_x, start_y, end_x, end_y, safety_thickness)

    count = 0

    mask_height, mask_width = mask.shape[:2]  # mask의 높이와 너비

    for x, y in line_points:
        # 유효한 범위 내의 좌표만 처리
        if 0 <= y < mask_height and 0 <= x < mask_width and end_x != 0 or end_y != 0:
            count += mask[y, x] / 255
            if mask[y, x] == 255:
                # frame에 점을 그림
                cv2.circle(frame, (x, y), radius=1, color=(0, 0, 200), thickness=-1)

    print(f"count = ", count)

    if count > area_threshold:
        border_before_hole_detected = True
    else:
        border_before_hole_detected = False
    
    return border_before_hole_detected

def get_line_points(x1, y1, x2, y2):
    """Bresenham 알고리즘을 사용하여 두 점 사이의 좌표들을 계산합니다."""
    points = []
    dx = abs(x2 - x1)
    dy = -abs(y2 - y1)

    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1

    err = dx + dy
    x, y = x1, y1

    while True:
        points.append((x, y))
        if x == x2 and y == y2:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return points

def get_line_points_with_thickness(x1, y1, x2, y2, thickness=1):
    """Bresenham 알고리즘을 사용하여 두 점 사이의 좌표를 계산하고, 선의 굵기를 추가합니다."""
    points = []
    line_points = get_line_points(x1, y1, x2, y2)

    for (x, y) in line_points:
        # 중심 점을 기준으로 주변에 사각형을 그려 선의 굵기를 추가
        for i in range(-thickness, thickness + 1):
            for j in range(-thickness, thickness + 1):
                points.append((x + i, y + j))

    return points

def ball_at_center(cx, cy, limits):
    if cx <= limits[0]:
        TX_num = 14 # 
    elif cx >= limits[1]:
        TX_num = 13
    elif cy <= limits[2]:
        TX_num = 11
    elif cy >= limits[3]:
        TX_num = 12
    else:
        TX_num = 0
    return TX_num

def get_hole_distance(cy, head_angle_z):
    hole_distance = int(H_View_size - cy + head_angle_z * 1.5)
    return hole_distance


motion_dict = {
    (90, -0): 36,
    (90, -15): 37, 
    (90, -30): 38,
    (90, -45): 39,
    (0, -0): 40,
    (0, -15): 41, 
    (0, -30): 42,
    (0, -45): 43,
    (0, -80): 31,
    (-90, 0): 44,
    (-90, -15): 45,
    (-90, -30): 46,
    (-90, -45): 47,
}                        # (xy_angle, z_angle)

# **************************************************
# **************************************************
# **************************************************
if __name__ == '__main__':

    #-------------------------------------
    print ("-------------------------------------")
    print ("(2020-1-20) mini CTS5 Program.  MINIROBOT Corp.")
    print ("-------------------------------------")
    print ("")
    os_version = platform.platform()
    print (" ---> OS " + os_version)
    python_version = ".".join(map(str, sys.version_info[:3]))
    print (" ---> Python " + python_version)
    opencv_version = cv2.__version__
    print (" ---> OpenCV  " + opencv_version)
    
   
   
    #-------------------------------------
    #---- user Setting -------------------
    #-------------------------------------
    W_View_size =  640  #320  #640
    #H_View_size = int(W_View_size / 1.777)
    H_View_size = int(W_View_size / 1.333)

    BPS =  4800  # 4800,9600,14400, 19200,28800, 57600, 115200
    serial_use = 1
    View_select = 0
    #-------------------------------------
    print(" ---> Camera View: " + str(W_View_size) + " x " + str(H_View_size) )
    print ("")
    print ("-------------------------------------")
    
    #-------------------------------------
    try:
        hsv_setting_read()
    except:
        hsv_setting_save()
        
        
    #-------------------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64,
                    help="max buffer size")
    args = vars(ap.parse_args())

    img = create_blank(320, 100, rgb_color=(0, 0, 255))
    
    cv2.namedWindow(Top_name)
    cv2.moveWindow(Top_name,0,0)
    
    cv2.createTrackbar('Hmax', Top_name, h_max[now_color], 255, Hmax_change)
    cv2.createTrackbar('Hmin', Top_name, h_min[now_color], 255, Hmin_change)
    cv2.createTrackbar('Smax', Top_name, s_max[now_color], 255, Smax_change)
    cv2.createTrackbar('Smin', Top_name, s_min[now_color], 255, Smin_change)
    cv2.createTrackbar('Vmax', Top_name, v_max[now_color], 255, Vmax_change)
    cv2.createTrackbar('Vmin', Top_name, v_min[now_color], 255, Vmin_change)
    cv2.createTrackbar('Min_Area', Top_name, min_area[now_color], 255, min_area_change)
    cv2.createTrackbar('Color_num', Top_name,color_num[now_color], 4, Color_num_change)

    Trackbar_change(now_color)

    draw_str3(img, (15, 25), 'MINIROBOT Corp.')
    draw_str2(img, (15, 45), 'space: Fast <=> Video and Mask.')
    draw_str2(img, (15, 65), 's, S: Setting File Save')
    draw_str2(img, (15, 85), 'Esc: Program Exit')
    


    cv2.imshow(Top_name, img)
    #---------------------------
    # if not args.get("video", False):
    #     camera = cv2.VideoCapture(0)
    # else:
    #     camera = cv2.VideoCapture(args["video"])
    camera = PiCamera()
    camera.awb_mode = 'sunlight'
    #---------------------------
    # camera.set(3, W_View_size)
    # camera.set(4, H_View_size)
    # camera.set(5, 40)
    camera.resolution = (W_View_size, H_View_size)
    camera.framerate = 40

    time.sleep(0.5)
    #---------------------------
        
    #---------------------------
    # (grabbed, frame) = camera.read()
    rawCapture = PiRGBArray(camera, size=(W_View_size, H_View_size))
    camera.capture(rawCapture, format="bgr")
    frame = rawCapture.array
    grabbed = frame is not None and frame.size > 0
    frame = rawCapture.array
    
    draw_str2(frame, (5, 15), 'X_Center x Y_Center =  Area' )
    draw_str2(frame, (5, H_View_size - 5), 'View: %.1d x %.1d.  Space: Fast <=> Video and Mask.'
                      % (W_View_size, H_View_size))
    draw_str_height(frame, (5, int(H_View_size/2)), 'Fast operation...', 3.0 )
    mask = frame.copy()
    cv2.imshow('mini CTS5 - Video', frame )
    # cv2.imshow('mini CTS5 - Mask', mask)
    cv2.moveWindow('mini CTS5 - Mask',322 + W_View_size,36)
    cv2.moveWindow('mini CTS5 - Video',322,36)
    cv2.setMouseCallback('mini CTS5 - Video', mouse_move)

    #---------------------------
    if serial_use != 0:  # python3
    #if serial_use <> 0:  # python2.7
        BPS =  4800  # 4800,9600,14400, 19200,28800, 57600, 115200
        #---------local Serial Port : ttyS0 --------
        #---------USB Serial Port : ttyAMA0 --------
        serial_port = serial.Serial('/dev/ttyS0', BPS, timeout=0.01)
        serial_port.flush() # serial cls
        time.sleep(0.5)
    
        serial_t = Thread(target=RX_Receiving, args=(serial_port,))
        serial_t.daemon = True
        serial_t.start()
        
    # First -> Start Code Send 
    TX_data(serial_port, 250)
    TX_data(serial_port, 250)
    TX_data(serial_port, 250)
    
    old_time = clock()

    View_select = 0
    msg_one_view = 0
    
    ball_detected = False
    hole_detected = False
    border_before_hole_detected = False
    head_angle = (0, -30)

        # Byoungseo 20240823
    center_region_width = 200
    left_region_limit = int(W_View_size / 2 - center_region_width / 2)
    right_region_limit = int(W_View_size / 2 + center_region_width / 2)

    bottom_region_width = 160
    bottom_region_limit = H_View_size - bottom_region_width

    ball_at_center_range = 80
    ball_at_center_left_limit = int(W_View_size / 2 - ball_at_center_range / 2)
    ball_at_center_right_limit = int(W_View_size / 2 + ball_at_center_range / 2)
    ball_at_center_top_limit = int(H_View_size / 2 - ball_at_center_range / 2)
    ball_at_center_bottom_limit = int(H_View_size / 2 + ball_at_center_range / 2)

    ball_at_point_range = 60
    ball_at_point_left_limit = int(W_View_size / 2 - ball_at_point_range / 2 + 80)
    ball_at_point_right_limit = int(W_View_size / 2 + ball_at_point_range / 2 + 80)
    ball_at_point_top_limit = int(H_View_size / 2 - ball_at_point_range / 2)
    ball_at_point_bottom_limit = int(H_View_size / 2 + ball_at_point_range / 2)

    hole_center_region_width = 100
    hole_left_region_limit = int(W_View_size / 2 - hole_center_region_width / 3)
    hole_right_region_limit = int(W_View_size / 2 + hole_center_region_width / 3)



    status = 0
    # 0: Finding Ball
    # 1: Walking toward the Ball -> 공 높이에 따라 고개 숙이기
    # 2: Ball at center
    # 3: Finding Hole
    # 4: Hole at hit point
    # 5: Ball at hit point
    # 6: Hitting the Ball
    # 7: Tracking Ball -> 왼쪽으로 고개 돌린 상태에서 고개 숙이기

    TX_num = motion_dict[head_angle]
    # 29: middle down
    # 31: extreme down

    hole_success = False
    ball_success = False

    hole_distance = 0

    TX_data(serial_port, TX_num)

    delay = 0

    only_video = True # False

    hit_cnt = 0

    hit_direction = 0  # 0: left, 1: right

    # -------- Main Loop Start --------
    while True:
        # grab the current frame
        # (grabbed, frame) = camera.read()
        rawCapture = PiRGBArray(camera, size=(W_View_size, H_View_size))
        camera.capture(rawCapture, format="bgr")
        frame = rawCapture.array
        grabbed = frame is not None and frame.size > 0

        if args.get("video") and not grabbed:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)    # HSV => YUV
        mask = cv2.inRange(hsv, hsv_Lower, hsv_Upper)
        
        hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
        hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])

        mask0 = cv2.inRange(hsv, (h_min[0], s_min[0], v_min[0]), (h_max[0], s_max[0], v_max[0]))
        mask1 = cv2.inRange(hsv, (h_min[1], s_min[1], v_min[1]), (h_max[1], s_max[1], v_max[1]))
        mask2 = cv2.inRange(hsv, (h_min[2], s_min[2], v_min[2]), (h_max[2], s_max[2], v_max[2]))
        mask3 = cv2.inRange(hsv, (h_min[3], s_min[3], v_min[3]), (h_max[3], s_max[3], v_max[3]))
        mask4 = cv2.inRange(hsv, (h_min[4], s_min[4], v_min[4]), (h_max[4], s_max[4], v_max[4]))
        
        #mask = cv2.erode(mask, None, iterations=1)
        #mask = cv2.dilate(mask, None, iterations=1)
        #mask = cv2.GaussianBlur(mask, (3, 3), 2)  # softly
        
        '''
        cnts0 = cv2.findContours(mask0.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts3 = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts4 = cv2.findContours(mask4.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        '''
        
        center = None

        hole_detected, hole_area, (cx_hole, cy_hole), closing = hole_detecting(frame, mask1, hsv, min_area_hole, max_area_hole, min_circularity_hole, max_aspect_ratio_hole)
        
        # 241012
        cv2.imshow('Hole Detection with Pole Ignoring', closing)

        X_Size, Y_Size, X_255_point, Y_255_point, cx_ball, cy_ball, ball_detected, Area, Angle = ball_detecting(mask0)

        # border_before_hole_detected = border_before_hole_detecting(frame, mask3, cx_hole, cy_hole, W_View_size, H_View_size, 400, 10)

        Frame_time = (clock() - old_time) * 1000.
        old_time = clock()
           
        if View_select == 0:    # Fast operation 
            # print(" " + str(W_View_size) + " x " + str(H_View_size) + " =  %.1f ms" % (Frame_time ))
            #temp = Read_RX
            pass
            
        elif View_select == 1:  # Debug
            
            if msg_one_view > 0:
                msg_one_view = msg_one_view + 1
                cv2.putText(frame, "SAVE!", (50, int(H_View_size / 2)),
                            cv2.FONT_HERSHEY_PLAIN, 5, (255, 255, 255), thickness=5)
                
                if msg_one_view > 10:
                    msg_one_view = 0                
                                
            draw_str2(frame, (3, 15), 'X: %.1d, Y: %.1d, status: %.1d, ball_detected: %.1d, hole_detected: %.1d, TX_num: %.1d, hit_direction: %.1d' 
                      % (X_255_point, Y_255_point, status, ball_detected, hole_detected, TX_num, hit_direction))
            draw_str2(frame, (3, H_View_size - 5), 'View: %.1d x %.1d Time: %.1f ms  Space: Fast <=> Video and Mask.'
                      % (W_View_size, H_View_size, Frame_time))

            if not only_video:  # for hsv select

                if status == 1:
                    cv2.line(frame, (0, bottom_region_limit), (W_View_size, bottom_region_limit), (0, 0, 255), 3)

                if status == 2:
                    cv2.rectangle(frame, (ball_at_center_left_limit, ball_at_center_top_limit), (ball_at_center_right_limit, ball_at_center_bottom_limit), (255, 255, 255), 2)

                if status == 4:
                    tuned_left_limit = hole_left_region_limit + int((H_View_size - get_hole_distance(cy_hole, head_angle[1])) * 0.3)
                    tuned_right_limit = hole_right_region_limit + int((H_View_size - get_hole_distance(cy_hole, head_angle[1])) * 0.3)
                    cv2.line(frame, (tuned_left_limit, 0), (tuned_left_limit, H_View_size), (0, 0, 255), 3)
                    cv2.line(frame, (tuned_right_limit, 0), (tuned_right_limit, H_View_size), (0, 0, 255), 3)
                    cv2.line(frame, (0, H_View_size - 100), (W_View_size, H_View_size - 100), (155, 155, 0), 3)
                    cv2.line(frame, (0, H_View_size - 200), (W_View_size, H_View_size - 200), (255, 255, 0), 3)

                if status == 5:
                    cv2.rectangle(frame, (ball_at_point_left_limit, ball_at_point_top_limit), (ball_at_point_right_limit, ball_at_point_bottom_limit), (255, 255, 255), 2)

                if status == 6:
                    draw_str2(frame, (3, 30), 'hole_distance: %.1d' % (hole_distance))
                    
                if status == 11:
                    cv2.line(frame, (W_View_size // 2, H_View_size), (cx_hole, cy_hole), 5)


                if not ball_detected and status <= 1:
                    status = 0
                    
                
                if delay == 0:

                    # Action by Status
                    if status == 0:         # 0: Finding Ball
                        if TX_num == 0:
                            TX_num = 33
                            delay = 5
                        else:
                            # now_color = 0
                            if ball_detected:
                                status = 1
                                TX_num = 0
                                delay = 5
                                if hole_detected and cx_hole > cx_ball and hit_cnt > 0: # ball is on the left of the hole
                                    hit_direction = 1
                                else:                                   # ball is on the right of the hole
                                    hit_direction = 0
                            else:
                                if hit_direction == 0:  # left hit
                                    TX_num = 22      # turn left
                                    delay = 5
                                else:   # right hit
                                    TX_num = 24      # turn right
                                    delay = 5
                        
                    elif status == 1:        # 1: Walking towards the Ball
                        if cx_ball <= left_region_limit:        # ball is at the left side
                            TX_num = 1                          # turn left
                        elif cx_ball >= right_region_limit:     # ball is at the right side
                            TX_num = 3                          # turn right
                        else:                                   # ball is at the middle
                            # if TX_num in [1, 3]:
                            #     delay = 3                       # delay for swing by rotation           
                            if cy_ball < bottom_region_limit:   # ball is not close enough
                                TX_num = 11                     # step forward
                            else:                               # ball is close enough
                                if head_angle[1] > -30:
                                    head_angle = (0, head_angle[1] - 15)
                                    TX_num = motion_dict[head_angle]
                                    delay = 10
                                else:
                                    status = 2
                                    TX_num = 0
                                    delay = 10
                    
                    elif status == 2:       # 2: Ball at center
                        if TX_num == 0:
                            head_angle = (0, -80)
                            TX_num = motion_dict[head_angle]           # head front down
                            delay = 5
                        else:
                            limits = [ball_at_center_left_limit, ball_at_center_right_limit, ball_at_center_top_limit, ball_at_center_bottom_limit]
                            TX_num = ball_at_center(cx_ball, cy_ball, limits)
                            delay = 5
                            if TX_num == 0:
                                status = 3

                    elif status == 3:       # 3: Finding Hole
                        if TX_num == 0:
                            head_angle = (90 if hit_direction == 0 else -90, -0 if hit_cnt == 0 else -30)
                            TX_num = motion_dict[head_angle]            # head left up
                            delay = 5
                        elif TX_num in [9, 7, motion_dict[head_angle]]:
                            if hit_direction == 0:
                                TX_num = 14    # step left
                            else:
                                TX_num = 13    # step right
                            delay = 2
                        elif TX_num in [14, 13]: 
                            if hit_direction == 0:
                                TX_num = 9      # turn right
                            else:
                                TX_num = 7      # turn left
                            delay = 2
                        if hole_detected and TX_num in [9, 7, 14, 13]:
                            status = 4
                            TX_num = 0
                            delay = 10

                    elif status == 4:      # Hole at hit point
                        if TX_num == 0:    
                            head_angle = (90 if hit_direction == 0 else -90, -0 if hit_cnt == 0 else -30)
                            TX_num = motion_dict[head_angle]             # head left up
                            delay = 5
                        else:
                            if not hole_detected:
                                TX_num = 0
                                status = 3
                                delay = 5
                            elif cx_hole <= tuned_left_limit:       # hole is at the left side
                                TX_num = 1                          # turn right
                                ball_success = False
                            elif cx_hole >= tuned_right_limit:      # hole is at the right side
                                TX_num = 3                          # turn left
                                ball_success = False
                            else:
                                hole_success = True
                                hole_distance = get_hole_distance(cy_hole, head_angle[1])
                                status = 5
                                TX_num = 0
                                delay = 5
                                
                    elif status == 5:       # 5: Ball at hit point
                        if TX_num == 0:
                            head_angle = (0, -80)
                            TX_num = motion_dict[head_angle]
                            delay = 5
                        else:
                            limits = [ball_at_point_left_limit, ball_at_point_right_limit, ball_at_point_top_limit, ball_at_point_bottom_limit]
                            TX_num = ball_at_center(cx_ball, cy_ball, limits)
                            delay = 5
                            if TX_num == 0 and (not hole_success or not ball_success):
                                ball_success = True
                                status = 4
                            elif TX_num == 0 and (hole_success and ball_success):
                                # status = 11
                                status = 6
                            else:
                                ball_success = False
                                hole_success = False
                
                    elif status == 6:       # 6: Hitting the Ball
                        if TX_num == 0:
                            if hit_direction == 0:
                                if hole_distance > 200:
                                    TX_num = 2     # hit the ball
                                elif hole_distance > 100:
                                    TX_num = 34
                                else:
                                    TX_num = 35
                            else:
                                TX_num = 5
                            delay = 20
                        else:
                            hit_cnt += 1
                            TX_num = 0
                            delay = 5
                            status = 7

                    elif status == 7:       # 7: Tracking Ball
                        head_angle_x = 90 if hit_direction == 0 else -90
                        if TX_num == 0:
                            head_angle = (head_angle_x, -0)
                            TX_num = motion_dict[head_angle]
                            delay = 5
                        else:
                            if ball_detected:
                                delay = 5
                                TX_num = 0
                                status = 0
                            else:
                                if head_angle[1] < -45:
                                    head_angle = (head_angle_x, -0)         # Todo: Exception
                                else:
                                    head_angle = (head_angle_x, head_angle[1] - 15)
                                TX_num = motion_dict[head_angle]
                                delay = 5

                    # elif status == 11:
                    #     if TX_num == 0:    
                    #         TX_num = 36    # head up, left
                    #         delay = 5
                    #     if border_before_hole_detected:
                    #         status = 6
                    #     print(border_before_hole_detected)

                    print("TX_num: {}".format(TX_num))
                    TX_data(serial_port, TX_num)
                else:
                    TX_data(serial_port, 0)
                    delay = delay - 1
                      
            #------mouse pixel hsv -------------------------------
            mx2 = mx
            my2 = my
            if mx2 < W_View_size and my2 < H_View_size:
                pixel = hsv[my2, mx2]
                set_H = pixel[0]
                set_S = pixel[1]
                set_V = pixel[2]
                pixel2 = frame[my2, mx2]
                if my2 < (H_View_size / 2):
                    if mx2 < (W_View_size / 2):
                        x_p = -30
                    elif mx2 > (W_View_size / 2):
                        x_p = 60
                    else:
                        x_p = 30
                    draw_str2(frame, (mx2 - x_p, my2 + 15), '-HSV-')
                    draw_str2(frame, (mx2 - x_p, my2 + 30), '%.1d' % (pixel[0]))
                    draw_str2(frame, (mx2 - x_p, my2 + 45), '%.1d' % (pixel[1]))
                    draw_str2(frame, (mx2 - x_p, my2 + 60), '%.1d' % (pixel[2]))
                else:
                    if mx2 < (W_View_size / 2):
                        x_p = -30
                    elif mx2 > (W_View_size / 2):
                        x_p = 60
                    else:
                        x_p = 30
                    draw_str2(frame, (mx2 - x_p, my2 - 60), '-HSV-')
                    draw_str2(frame, (mx2 - x_p, my2 - 45), '%.1d' % (pixel[0]))
                    draw_str2(frame, (mx2 - x_p, my2 - 30), '%.1d' % (pixel[1]))
                    draw_str2(frame, (mx2 - x_p, my2 - 15), '%.1d' % (pixel[2]))
            #----------------------------------------------
            
            cv2.imshow('mini CTS5 - Video', frame )
            # cv2.imshow('mini CTS5 - Mask', mask)
            cv2.imshow('mini CTS5 - Mask0', mask0)
            cv2.imshow('mini CTS5 - Mask1', mask1)
            cv2.imshow('mini CTS5 - Mask2', mask2)
            cv2.imshow('mini CTS5 - Mask3', mask3)
            # cv2.imshow('mini CTS5 - Mask4', mask4)

        key = 0xFF & cv2.waitKey(1)
        
        if key == 27:  # ESC  Key
            TX_data(serial_port, 0)
            break
        elif key == ord(' '):  # spacebar Key
            if View_select == 0:
                View_select = 1
            else:
                View_select = 0
        elif key == ord('s') or key == ord('S'):  # s or S Key:  Setting valus Save
            hsv_setting_save()
            msg_one_view = 1
        elif key == ord('h'):
            hole_detected = True
            

    # cleanup the camera and close any open windows
    receiving_exit = 0
    time.sleep(0.5)
    
    camera.close()
    cv2.destroyAllWindows()
