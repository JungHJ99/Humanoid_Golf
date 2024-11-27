# -*- coding: utf-8 -*-
# main_241109_pbs_01.py

# Done:
# Approaching Shot 시 hit_direction 판단
# Approaching Shot 시 고개 좌 or 우로 살짝 회전해서 시야 확보

# ToDo:
# 화살표 무시 (hole_detecting, near_hole_detecting)
# 홀인 세리모니 인식 (홀 가까운 경우, 홀 먼 경우)

import platform
import numpy as np
import argparse
import cv2
import serial
import time
import sys
import threading
from threading import Thread
import csv
import math
import argparse

from picamera import PiCamera
from picamera.array import PiRGBArray

X_255_point = 0
Y_255_point = 0
X_Size = 0
Y_Size = 0
Area = 0
Angle = 0
motion_finish_event = threading.Event()
#-----------------------------------------------
Top_name = 'mini CTS5 setting'
hsv_Lower = 0
hsv_Upper = 0

hsv_Lower0 = 0
hsv_Upper0 = 0

hsv_Lower1 = 0
hsv_Upper1 = 0

color_cnt = 7

#----------- 
# Mask0: pink_ball
# Mask1: yellow_outer_hole
# Mask2: black_inner_hole
# Mask3: light_green_field
# Mask4: dark_green_boundary
# Mask5: yellow_near_hole
#----------- 
# mask_list at 하늘6단지 헬스장
# color_num = [   0,  1,  2,  3,  4, 5]
# h_max =     [ 179,240, 140,200,120, 220]
# h_min =     [  86,0,  0, 86, 40, 170]

# s_max =     [ 121,76,130,111,140, 60]
# s_min =     [ 100, 0,85, 70, 103, 20]
    
# v_max =     [ 255,175,180,121,115,170]
# v_min =     [ 180, 0,100, 70, 67, 130]
    
# min_area =  [  3, 30, 50, 10, 10, 50]
#----------- 
# mask_list at 하늘 테라스, 18시
# color_num = [  0,  1,  2,  3,  4,  5, 6]
# h_max =     [150,189,100,152, 75,190, 152]
# h_min =     [  0,145,  0, 50, 28,142, 50]

# s_max =     [180, 59,138, 98,140, 73, 98]
# s_min =     [ 90, 38, 56, 46, 94, 33, 46]
    
# v_max =     [255,244,181,158,144,255, 158]
# v_min =     [177,  0, 53, 70,  0,  0, 70]
    
# min_area =  [  3, 30, 50, 10, 10, 50, 50]
#----------- 
# # mask_list at 하늘 테라스, 13시
# color_num = [  0,  1,  2,  3,  4, 5]
# h_max =     [151,240,140,200,244,220]
# h_min =     [  0,  0,  0, 50, 40,106]

# s_max =     [180,76,130,111,140, 86]
# s_min =     [100, 0, 85, 70,119, 20]
    
# v_max =     [255,175,180,121,115,170]
# v_min =     [170,  0,100, 70, 87,130]
    
# min_area =  [  3, 30, 50, 10, 10, 50]
#----------- 
# mask_list at 쌍둥이 방
# color_num = [   0,  1,  2,  3,  4, 5]
# h_max =     [ 179,240, 140,200,120, 220]
# h_min =     [  86,170,  0, 86, 40, 170]

# s_max =     [ 121,76,130,111,140, 60]
# s_min =     [ 100, 0,85, 70, 103, 20]
    
# v_max =     [ 255,180,180,121,115,230]
# v_min =     [ 180, 0,100, 70, 67, 140]
    
# min_area =  [  3, 30, 50, 10, 10, 50]
#-----------# 
# 0 -> 핑크 볼 
# 1 -> 노랑 홀
# 2 -> 검정 홀
# 5 -> 노랑 니어홀
# 7 -> 회색 벙커
color_num = [   0,  1,  2,  3,  4,  5,  6, 7]
h_max =     [247, 244, 108, 195, 83, 255, 212, 212]
h_min =     [107, 124, 32, 139, 26, 199, 76, 76]
s_max =     [158, 89, 155, 122, 140, 91, 141, 141]
s_min =     [110, 39, 66, 57, 110, 33, 46, 46]
v_max =     [255, 184, 155, 126, 112, 170, 111, 111]
v_min =     [151, 90, 182, 52, 0, 120, 47, 47]
min_area =  [3, 20, 50, 11, 10, 50, 50, 20]
#----------- 
# mask_list at 병서네 헬스장
# 0,150,0,180,90,255,177,3
# 1,224,124,69,39,244,0,30
# 2,100,0,145,56,181,53,50
# 3,154,80,123,57,108,66,10
# 4,75,29,140,94,144,0,10
# 5,200,160,73,33,255,0,50
# 6,75,29,140,94,144,0,10
#----------- 
#par3
# 0,247,107,158,110,255,151,3
# 1,244,124,89,39,184,90,20
# 2,147,24,147,78,146,82,50
# 3,195,139,122,57,126,52,11
# 4,83,26,140,110,112,0,10
# 5,255,169,91,33,170,120,50
# 6,212,76,141,46,111,47,50
# 7,201,164,127,74,145,112,41

# par4
# 0,247,107,158,110,255,151,3
# 1,244,124,89,39,184,90,20
# 2,147,24,147,78,146,82,50
# 3,195,63,122,57,126,52,11
# 4,83,26,140,110,112,0,10
# 5,255,169,91,33,170,120,50
# 6,212,76,141,46,111,47,50
# 7,201,164,127,74,145,112,41



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
    global motion_finish_event

    receiving_exit = 1
    while True:
        if receiving_exit == 0:
            break
        time.sleep(threading_Time)
        
        while ser.inWaiting() > 0:
            result = ser.read(1)
            RX = ord(result)
            print ("RX=" + str(RX))
            if RX == 38:
                motion_finish_event.set()
# *************************
def motion_finished():
    if motion_finish_event.is_set():
        motion_finish_event.clear()  # 이벤트 초기화
        return True
    else:
        return False
# *************************
def GetLengthTwoPoints(XY_Point1, XY_Point2):
    return math.sqrt( (XY_Point2[0] - XY_Point1[0])**2 + (XY_Point2[1] - XY_Point1[1])**2 )
# *************************
def FYtand(dec_val_v ,dec_val_h):
    return ( math.atan2(dec_val_v, dec_val_h) * (180.0 / math.pi))
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

    # GaussianBlurW
    # blurred_image = cv2.GaussianBlur(mask, (5, 5), 0)

    # Morph Close
    kernel = np.ones((25, 25), np.uint8)    # kernel = np.ones((20, 20), np.uint8)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # kenel 크기의 작은 구멍을 메움 / 2cm 폴대 무시하도록 kernel 키움

    # 일정 크기 이상인 노란색 면적의 윤곽선 반환
    contours, _ = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    hole_detected = False
    largest_ellipse = None
    largest_contour = None
    largest_area = 0
    largest_width = 0
    largest_height = 0
    cX, cY, cR = 0, 0, 0
    x1, x2, y1, y2 = 0, 0, 0, 0
    x_min = float('inf')
    x_max = float('-inf')

    largest_cX, largest_cY, largest_cR = 0, 0, 0
    largest_x1, largest_x2, largest_y1, largest_y2, largest_x_min, largest_x_max, largest_h_mean, largest_s_mean, largest_v_mean = 0, 0, 0, 0, 0, 0, 0, 0, 0

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

                x, y, w, h = cv2.boundingRect(cnt)  # 각 윤곽선의 경계 상자
                x_min = x
                x_max = x + w
                y_min = y
                y_max = y + h
            
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
                    largest_width = x_max - x_min
                    largest_height = y_max - y_min
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
    return hole_detected, largest_area, largest_width, largest_height, (largest_cX, largest_cY), closing

min_area_near_hole = 500
max_area_near_hole = 500000

def near_hole_detecting(frame, mask, hsv, min_area_near_hole, max_area_near_hole):
    # 윤곽선 찾기 (mask 이미지에서 바로 찾기)
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largest_contour = None
    largest_area = 0
    largest_cX, largest_cY = 0, 0
    near_hole_detected = False

    # 컨투어 중 가장 큰 윤곽선의 중심 좌표 찾기
    for cnt in contours:
        contour_area = cv2.contourArea(cnt)

        if contour_area >= min_area_near_hole and contour_area <= max_area_near_hole:
            if contour_area > largest_area:
                largest_area = contour_area
                largest_contour = cnt

                # 중심 좌표 계산
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    largest_cX = int(M["m10"] / M["m00"])
                    largest_cY = int(M["m01"] / M["m00"])
                    near_hole_detected = True

    if near_hole_detected:
        cv2.drawContours(frame, [largest_contour], -1, (200, 200, 0), 2)

    # 가장 큰 컨투어가 있을 경우, 해당 중심 좌표를 반환
    return near_hole_detected, (largest_cX, largest_cY)

def corner_detecting(frame, maskf, maskb):
    corner_detected = False
    cx, cy = 0, 0
    max_mean_roif = 0
    roi_num = 30  # 주변 영역 크기
    f_thr = 170  # 코너 주변 필드 비율 임계값
    b_thr = 30  # 코너 주변 테두리 비율 임계값
    g_from_c = 170 # 목표점 x좌표를 위한 오프셋
    goal_point_x = 0 # 목표점의 x좌표

    # ORB 설정
    orb = cv2.ORB_create()
    keypoints = orb.detect(maskf, None)

    for idx, kp in enumerate(keypoints): 
        x, y = int(kp.pt[0]), int(kp.pt[1])
        roif = maskf[y - roi_num:y + roi_num + 1, x - roi_num:x + roi_num + 1]
        roib = maskb[y - roi_num:y + roi_num + 1, x - roi_num:x + roi_num + 1]  # 테두리
        mean_roif = np.mean(roif)
        mean_roib = np.mean(roib)

        
        if mean_roif > f_thr and mean_roib > b_thr:  # 주변 필드, 테두리 비율이 임계값 이상인 경우
            if mean_roif > max_mean_roif:  # 가장 주변 흰색 비율이 큰 점 선택
                max_mean_roif = mean_roif
                cx, cy = x, y
                corner_detected = True

                goal_point_x = cx - g_from_c
                
    # 코너와 목표 지점 표시
    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
    cv2.circle(frame, (goal_point_x, cy), 5, (0, 0, 255), -1)

    return corner_detected, (cx, cy), goal_point_x

min_area_bunker = 50
max_area_bunker = 500000
def bunker_detecting(frame, mask, hsv, min_area_bunker, max_area_bunker):
    # 윤곽선 찾기 (mask 이미지에서 바로 찾기)
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largest_contour = None
    largest_area = 0
    largest_cX, largest_cY = 0, 0
    bunker_detected = False

    # 컨투어 중 가장 큰 윤곽선의 중심 좌표 찾기
    for cnt in contours:
        contour_area = cv2.contourArea(cnt)

        if contour_area >= min_area_bunker and contour_area <= max_area_bunker:
            if contour_area > largest_area:
                largest_area = contour_area
                largest_contour = cnt

                # 중심 좌표 계산
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    largest_cX = int(M["m10"] / M["m00"])
                    largest_cY = int(M["m01"] / M["m00"])
                    bunker_detected = True

    if bunker_detected:
        cv2.drawContours(frame, [largest_contour], -1, (0, 0, 0), 2)

    # 가장 큰 컨투어가 있을 경우, 해당 중심 좌표를 반환
    return bunker_detected, (largest_cX, largest_cY)

def ball_at_hit_point(cx, cy, limits):

    if cy <= limits[2]:
        TX_num = 10     # 종종전진_골프
    elif cy >= limits[3]:
        TX_num = 48     # 종종후진_골프
    elif cx <= limits[0]:
        TX_num = 15     # 왼쪽옆으로20연속_골프
    elif cx >= limits[1]:
        TX_num = 20     # 오른쪽옆으로20연속_골프
    else:
        TX_num = 0
    return TX_num

def near_hole_at_hit_point(cx, cy, limits):
    if cx <= W_View_size / 2:
        hit_direction = 0       # hit left
        if cy <= limits[2]:     # 홀이 공보다 위에 있을 때
            TX_num = 3          # TX3: 오른쪽턴5_골프
        elif cy >= limits[3]:   # 홀이 공보다 아래에 있을 때
            TX_num = 1          # TX1: 왼쪽턴5_골프
        else:
            TX_num = 0
    else:
        hit_direction = 1       # hit right
        if cy <= limits[2]:     # 홀이 공보다 아래에 있을 때
            TX_num = 1          # TX1: 왼쪽턴5_골프
        elif cy >= limits[3]:   # 홀이 공보다 위에 있을 때
            TX_num = 3          # TX3: 오른쪽턴5_골프
        else:
            TX_num = 0
    return TX_num, hit_direction

def get_hole_distance(hole_width):
    return 264 - hole_width

def get_screen_arm_length(hole_width):
    hole_real_width = 15
    arm_real_length = 15
    return int(arm_real_length * hole_width / hole_real_width)

motion_dict = {
    (90, -0): 36,   # TX36: 머리왼쪽90도하향0도
    (90, -15): 37,  # TX37: 머리왼쪽90도하향15도
    (90, -30): 38,  # TX38: 머리왼쪽90도하향30도
    (90, -45): 39,  # TX39: 머리중앙하향45도
    (0, -0): 40,    # TX40: 머리중앙하향0도
    (0, -15): 41,   # TX41: 머리중앙하향15도
    (0, -30): 42,   # TX42: 머리중앙하향30도
    (0, -45): 43,   # TX43: 머리중앙하향45도
    (0, -80): 31,   # TX31: 머리중앙하향80도
    (-90, 0): 44,   # TX44: 머리오른쪽90도하향0도
    (-90, -15): 45, # TX45: 머리오른쪽90도하향15도
    (-90, -30): 46, # TX46: 머리오른쪽90도하향30도
    (-90, -45): 47, # TX47: 머리오른쪽90도하향45도
    (15, -80): 49,  # TX49: 머리왼쪽15도하향80도
    (-15, -80): 50, # TX50: 머리오른쪽15도하향80도
    (45, -45): 51, # TX51
    (-45, -45): 52, # TX52

}                   # (xy_angle, z_angle)

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
    ap.add_argument('--map', type=str, default='par3', help="Map name")
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
    cv2.createTrackbar('Color_num', Top_name,color_num[now_color], color_cnt, Color_num_change)

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

    msg_one_view = 0
    
    ball_detected = False
    hole_detected = False
    border_before_hole_detected = False

    near_hole_detected = False
    bunker_detected = False


        # Byoungseo 20240823
    center_region_width = 200
    left_region_limit = int(W_View_size / 2 - center_region_width / 2)
    right_region_limit = int(W_View_size / 2 + center_region_width / 2)

    bottom_region_width = 140
    bottom_region_limit = H_View_size - bottom_region_width

    ball_at_center_range = 120
    ball_at_center_left_limit = int(W_View_size / 2 - ball_at_center_range / 2)
    ball_at_center_right_limit = int(W_View_size / 2 + ball_at_center_range / 2)
    ball_at_center_upper_limit = int(H_View_size / 2 - ball_at_center_range / 2)
    ball_at_center_lower_limit = int(H_View_size / 2 + ball_at_center_range / 2)

    ball_at_hit_point_range = 40
    ball_at_hit_point_left_limit = int(W_View_size / 2 - ball_at_hit_point_range / 2 + 80)
    ball_at_hit_point_right_limit = int(W_View_size / 2 + ball_at_hit_point_range / 2 + 80)
    ball_at_hit_point_upper_limit = int(H_View_size / 2 - ball_at_hit_point_range / 2 - 30)
    ball_at_hit_point_lower_limit = int(H_View_size / 2 + ball_at_hit_point_range / 2 - 30)

    ball_at_hit_point_range_par4 = 80
    ball_at_hit_point_left_limit_par4 = int(W_View_size / 2 - ball_at_hit_point_range_par4 / 2 + 80)
    ball_at_hit_point_right_limit_par4 = int(W_View_size / 2 + ball_at_hit_point_range_par4 / 2 + 80)
    ball_at_hit_point_upper_limit_par4 = int(H_View_size / 2 - ball_at_hit_point_range_par4 / 2 - 30)
    ball_at_hit_point_lower_limit_par4 = int(H_View_size / 2 + ball_at_hit_point_range_par4 / 2 - 30)

    hole_center_region_width = 50
    hole_left_region_limit = int(W_View_size / 2 - hole_center_region_width / 3)
    hole_right_region_limit = int(W_View_size / 2 + hole_center_region_width / 3)

    near_hole_at_hit_point_range = 60
    near_hole_at_hit_point_left_limit = int (0)
    near_hole_at_hit_point_right_limit = int (W_View_size)
    near_hole_at_hit_point_upper_limit = int(H_View_size / 2 - near_hole_at_hit_point_range / 2 - 50)
    near_hole_at_hit_point_lower_limit = int(H_View_size / 2 + near_hole_at_hit_point_range / 2 - 50)

    corner_center_region_width = 100
    corner_left_region_limit = int(W_View_size / 2 + 30)
    corner_right_region_limit = int(W_View_size / 2 + corner_center_region_width + 30)

    near_hole_center_region_width = 80
    near_hole_left_region_limit = int(W_View_size / 2 - near_hole_center_region_width)
    near_hole_right_region_limit = int(W_View_size / 2 + near_hole_center_region_width)

    status = 0
    # 0: Finding Ball
    # 1: Walking toward the Ball -> 공 높이에 따라 고개 숙이기
    # 2: Ball at center
    # 3: Finding Hole
        # 31: Approaching Shot
    # 4: Hole at hit point
    # 5: Ball at hit point
    # 6: Hitting the Ball
    # 7: Tracking Ball -> 왼쪽으로 고개 돌린 상태에서 고개 숙이기

    # 29: middle down
    # 31: extreme down

    goal_point_success = False
    ball_success = False

    hole_distance = 0

    near_hole_detected = False

    start_time = None
    duration_time = 0

    delay = 0
    delay_until = 0
    is_delay = False

    only_video = False

    hit_cnt = 0

    status_0_turn_cnt = 0

    hit_direction = 0  # 0: left, 1: right

    head_angle = (90, -15)

    after_hit_move1 = 12
    after_hit_move2 = 15

    hit_strength = 0

    TX_num = motion_dict[head_angle]

    TX_data(serial_port, TX_num)

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

        # mask3 = cv2.inRange(hsv, (h_min[3], s_min[3], v_min[3]), (h_max[3], s_max[3], v_max[3]))
        # kernel = np.ones((15, 15), np.uint8)
        # mask3 = cv2.morphologyEx(mask3, cv2.MORPH_CLOSE, kernel)

        # mask4 = cv2.inRange(hsv, (h_min[4], s_min[4], v_min[4]), (h_max[4], s_max[4], v_max[4]))
        # kernel = np.ones((3, 3), np.uint8)
        # mask4 = cv2.morphologyEx(mask4, cv2.MORPH_CLOSE, kernel)

        mask5 = cv2.inRange(hsv, (h_min[5], s_min[5], v_min[5]), (h_max[5], s_max[5], v_max[5]))

        # mask6 = cv2.inRange(hsv, (h_min[6], s_min[6], v_min[6]), (h_max[6], s_max[6], v_max[6]))
        # kernel = np.ones((15, 15), np.uint8)
        # mask6 = cv2.morphologyEx(mask6, cv2.MORPH_CLOSE, kernel)

        mask7 = cv2.inRange(hsv, (h_min[7], s_min[7], v_min[7]), (h_max[7], s_max[7], v_max[7]))
        
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


        
        hole_detected, hole_area, hole_width, hole_height, (cx_hole, cy_hole), closing = hole_detecting(frame, mask1, hsv, min_area_hole, max_area_hole, min_circularity_hole, max_aspect_ratio_hole)
        near_hole_detected, (cx_near_hole, cy_near_hole) = near_hole_detecting(frame, mask5, hsv, min_area_near_hole, max_area_near_hole)
        bunker_detected, (cx_bunker, cy_bunker) = bunker_detecting(frame, mask7, hsv, min_area_bunker, max_area_bunker)
        # corner_detected, (cx_corner, cy_corner), par4_goal_x = corner_detecting(frame, mask3, mask4)

        far_shot = hit_cnt == 0 or (args['map'] == 'par4' and hit_cnt <= 1)


        # 공을 보내야하는 포인트 지정
        if hit_cnt == 0 and args['map'] == 'par4':
            goal_point_detected = bunker_detected
            cx_goal_point = cx_bunker
            cy_goal_point = cy_bunker
        else:
            goal_point_detected = hole_detected
            cx_goal_point = cx_hole
            cy_goal_point = cy_hole
        
        # 241012
        cv2.imshow('Hole Detection with Pole Ignoring', closing)

        X_Size, Y_Size, X_255_point, Y_255_point, cx_ball, cy_ball, ball_detected, Area, Angle = ball_detecting(mask0)

        # border_before_hole_detected = border_before_hole_detecting(frame, mask3, cx_hole, cy_hole, W_View_size, H_View_size, 400, 10)

        Frame_time = (clock() - old_time) * 1000.
        old_time = clock()

        previous_View_select = View_select

        # duration_time 업데이트
        if  start_time is not None:
            duration_time = clock() - start_time
        else:
            duration_time = 0
           
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
            
            draw_str2(frame, (3, 15), 'duration_time: %.1d, X: %.1d, Y: %.1d, status: %.1d, hit_cnt: %.1d, hit_direction: %.1d, TX_num: %.1d' 
                      % (duration_time, X_255_point, Y_255_point, status, hit_cnt, hit_direction, TX_num))
            draw_str2(frame, (3, 30), 'ball_detected: %.1d, hole_detected: %.1d, near_hole_detected: %.1d' 
                      % (ball_detected, hole_detected, near_hole_detected))
            draw_str2(frame, (3, 45), 'ball_success: %.1d, goal_point_success: %.1d' 
                      % (ball_success, goal_point_success))
            draw_str2(frame, (3, H_View_size - 5), 'View: %.1d x %.1d Time: %.1f ms  Space: Fast <=> Video and Mask.'
                      % (W_View_size, H_View_size, Frame_time))

            if status == 1:
                # 하단 범위 표시
                cv2.line(frame, (0, bottom_region_limit), (W_View_size, bottom_region_limit), (0, 0, 255), 3)

            if status == 2:
                # 공이 중앙이 되도록 하는 범위 표시
                cv2.rectangle(frame, (ball_at_center_left_limit, ball_at_center_upper_limit), (ball_at_center_right_limit, ball_at_center_lower_limit), (255, 255, 255), 2)

            if status == 31:
                cv2.line(frame, (0, bottom_region_limit), (W_View_size, bottom_region_limit), (0, 0, 255), 3)
                cv2.line(frame, (0, int(cy_ball) + 20), (W_View_size, int(cy_ball) + 20), (255, 255, 255))
                cv2.line(frame, (0, int(cy_ball) - 20), (W_View_size, int(cy_ball) - 20), (255, 255, 255))
                cv2.rectangle(frame, (ball_at_hit_point_left_limit_par4, ball_at_hit_point_upper_limit_par4), (ball_at_hit_point_right_limit_par4, ball_at_hit_point_lower_limit_par4), (0, 0, 255), 2)

            if status == 4:
                if args['map'] == 'par4' and hit_cnt == 0:  # 파4 첫타일 때, 코너를 골 포인트로 인식
                    gp_left_region_limit = corner_left_region_limit
                    gp_right_region_limit = corner_right_region_limit
                elif not far_shot:
                    if hit_direction == 0:
                        gp_left_region_limit = hole_left_region_limit - 370
                        gp_right_region_limit = hole_right_region_limit - 370
                    else:
                        gp_left_region_limit = hole_left_region_limit + 370
                        gp_right_region_limit = hole_right_region_limit + 370
                else:
                    gp_left_region_limit = hole_left_region_limit
                    gp_right_region_limit = hole_right_region_limit

                # 홀이 중앙이 되도록 하는 범위 표시
                tuned_left_limit = gp_left_region_limit + get_screen_arm_length(hole_width) * (1 if hit_direction == 0 else -1)
                tuned_right_limit = gp_right_region_limit + get_screen_arm_length(hole_width) * (1 if hit_direction == 0 else -1)
                cv2.line(frame, (tuned_left_limit, 0), (tuned_left_limit, H_View_size), (0, 0, 255), 3)
                cv2.line(frame, (tuned_right_limit, 0), (tuned_right_limit, H_View_size), (0, 0, 255), 3)
                cv2.line(frame, (0, H_View_size - 100), (W_View_size, H_View_size - 100), (155, 155, 0), 3)
                cv2.line(frame, (0, H_View_size - 200), (W_View_size, H_View_size - 200), (255, 255, 0), 3)

            if status == 5:
                # 공이 타격 포인트가 되도록 하는 범위 표시
                if args['map'] == 'par4' and hit_cnt == 0:
                    cv2.rectangle(frame, (ball_at_hit_point_left_limit_par4, ball_at_hit_point_upper_limit_par4), (ball_at_hit_point_right_limit_par4, ball_at_hit_point_lower_limit_par4), (255, 255, 255), 2)
                else:
                    cv2.rectangle(frame, (ball_at_hit_point_left_limit, ball_at_hit_point_upper_limit), (ball_at_hit_point_right_limit, ball_at_hit_point_lower_limit), (255, 255, 255), 2)

            if status == 6:
                # hole distance 표시
                draw_str2(frame, (3, 30), 'hole_distance: %.1d' % (hole_distance))
                
            if status == 11:
                cv2.line(frame, (W_View_size // 2, H_View_size), (cx_hole, cy_hole), 5)

            current_time = clock()

            if not only_video:  # for hsv select

                if not is_delay:
                    if motion_finished():
                        delay_until = current_time + delay
                        is_delay = True
                        TX_data(serial_port, 0)
                    # if delay == 0:

                else:

                    # 4분 50초 지나면 approach_shot 후 ceremony (1순위)
                    if duration_time >= 290:
                        end_cnt = 1
                        if end_cnt == 1:
                            if hit_direction == 0:
                                TX_num = 35     # TX35: 골프_왼쪽으로_샷3
                            else:
                                TX_num = 5      # TX5: 골프_오른쪽으로_샷1
                            delay = 5
                            enc_cnt =+ -1
                        elif end_cnt == 0:
                            TX_data(serial_port, 23)    # TX23: 앉았다일어나기
                            break

                    # ball in hole : ceremony (2순위)
                    elif ball_detected and hole_detected and cx_hole - hole_width / 2 < cx_ball < cx_hole + hole_width / 2 and cy_hole - hole_height / 2 < cy_ball < cy_hole + hole_height / 2:
                        TX_data(serial_port, 23)    # TX23: 앉았다일어나기
                        break

                    # 공 잃어버리면 status 0: Finding Ball로 이동 (3순위)
                    elif not ball_detected and status <= 1:
                        status = 0

                    delay = 0.5 # default delay

                    if current_time >= delay_until:
                        is_delay = False

                        # Action by Status
                        if status == 0:         # 0: Finding Ball
                            ball_success = False
                            goal_point_success = False
                            if TX_num == 0:
                                head_angle = (0, head_angle[1])
                                TX_num = motion_dict[head_angle]
                                delay = 3
                            else:
                                # now_color = 0
                                if ball_detected:  
                                    status = 1
                                    TX_num = 0
                                    # delay = 5
                                    if far_shot == 0:
                                        hit_direction = 0

                                    elif hole_detected and cx_hole > cx_ball and hit_cnt > 0: # ball is on the left of the hole
                                        hit_direction = 1
                                    else:                                   # ball is on the right of the hole
                                        hit_direction = 0
                                else:
                                    if hit_direction == 0:  # left hit
                                        TX_num = 22      # TX22: 왼쪽턴45_골프
                                        # delay = 3
                                    else:   # right hit
                                        TX_num = 24      # TX24: 오른쪽턴45_골프
                                        # delay = 3

                                    status_0_turn_cnt += 1
                                    
                                    if status_0_turn_cnt > 9:
                                        status_0_turn_cnt = 0
                                        if head_angle[1] == -45:
                                            head_angle = (0, -15)
                                        else: 
                                            head_angle = (0, head_angle[1] - 15)
                                        TX_num = motion_dict[head_angle]
                                        delay = 3
                                    
                            
                        elif status == 1:        # 1: Walking towards the Ball
                            status_0_turn_cnt = 0
                            if cx_ball <= left_region_limit:        # ball is at the left side
                                TX_num = 1                          # TX1: 왼쪽턴5_골프
                            elif cx_ball >= right_region_limit:     # ball is at the right side
                                TX_num = 3                          # TX3: 오른쪽턴5_골프
                            else:                                   # ball is at the middle
                                # if TX_num in [1, 3]:
                                #     delay = 3                     # delay for swing by rotation           
                                if cy_ball < bottom_region_limit:   # ball is not close enough
                                    TX_num = 10                     # TX11: 연속전진_골프
                                else:                               # ball is close enough
                                    if head_angle[1] > -30:         # head angle down
                                        head_angle = (0, head_angle[1] - 15)
                                        TX_num = motion_dict[head_angle]
                                        delay = 3
                                    else:                           # go to status 2
                                        status = 2
                                        TX_num = 0
                                        # delay = 10
                        
                        elif status == 2:       # 2: Ball at center
                            if TX_num == 0:
                                head_angle = (0, -80)
                                TX_num = motion_dict[head_angle]           # head front down
                                delay = 3
                            else:
                                if not ball_detected:
                                    status = 21
                                    # delay = 5
                                else:
                                    limits = [ball_at_center_left_limit, ball_at_center_right_limit, ball_at_center_upper_limit, ball_at_center_lower_limit]
                                    TX_num = ball_at_hit_point(cx_ball, cy_ball, limits)    # step
                                    # delay = 3
                                    if TX_num == 0:
                                        if near_hole_detected and not far_shot:
                                            if cx_ball < cx_near_hole:
                                                hit_direction = 1
                                            else:
                                                hit_direction = 0
                                            status = 31
                                            TX_num = 0
                                            # delay = 10
                                        else:
                                            status = 3

                        elif status == 21:       # 2: Near Ball Lost
                            head_angle = (0, -45)
                            TX_num = motion_dict[head_angle]           # head front down
                            delay = 3
                            status = 0


                        elif status == 3:       # 3: Finding Hole
                            if TX_num == 0:
                                if far_shot:
                                    head_angle = (90 if hit_direction == 0 else -90, -15)
                                else:
                                    head_angle = (45 if hit_direction == 0 else -45, -45)
                                TX_num = motion_dict[head_angle]            # head left up
                                delay = 3
                            elif TX_num in [9, 7, motion_dict[head_angle]]:
                                if hit_direction == 0:  # hit left
                                    TX_num = 14         # TX14: 왼쪽옆으로70연속_골프
                                else:                   # hit right
                                    TX_num = 13         # TX13:오른쪽옆으로70연속_골프
                                delay = 1
                            elif TX_num in [14, 13]: 
                                if hit_direction == 0:  # hit left
                                    TX_num = 9          # TX9: 오른쪽턴20_골프
                                else:                   # hit right
                                    TX_num = 7          # TX7: 왼쪽턴20_골프
                                delay = 1
                            if goal_point_detected and TX_num in [9, 7, 14, 13]:
                                status = 4
                                TX_num = 0
                                # delay = 5


                        elif status == 31:      # 31: Approach Shot 1
                            if not near_hole_detected:
                                status = 2
                            else:
                                if cy_ball > bottom_region_limit:
                                    TX_num = 48
                                elif cy_near_hole < cy_ball - 20:     # hole이 near_ball보다 위
                                    if hit_direction == 0:          
                                        TX_num = 3                  # TX3:오른쪽턴5_골프
                                    else:
                                        TX_num = 1                  # TX1:왼쪽턴5_골프
                                    # delay = 0
                                elif cy_near_hole > cy_ball + 20:    # hole이 near_ball보다 아래
                                    if hit_direction == 0:
                                        TX_num = 1                  # TX1:왼쪽턴5_골프
                                    else:
                                        TX_num = 3                  # TX3:오른쪽턴5_골프
                                    # delay = 0
                                    
                                else:                               # hole이 near_ball과 같은 선상
                                    limits = [ball_at_hit_point_left_limit_par4, ball_at_hit_point_right_limit_par4, ball_at_hit_point_upper_limit_par4, ball_at_hit_point_lower_limit_par4]
                                    TX_num = ball_at_hit_point(cx_ball, cy_ball, limits)    # step
                                    # delay = 3
                                    if TX_num == 0:
                                        status = 6
                        
                        elif status == 4:       # 4: Hole at hit point
                            if TX_num == 0:
                                if far_shot:
                                    head_angle = (90 if hit_direction == 0 else -90, -15)
                                else:
                                    head_angle = (45 if hit_direction == 0 else -45, -45)

                                TX_num = motion_dict[head_angle]                # head left up
                                delay = 3
                            else:
                                if not goal_point_detected:
                                    status = 3
                                    TX_num = 0
                                elif cx_goal_point <= tuned_left_limit:         # hole is at the left side
                                    TX_num = 1                                  # TX1: 왼쪽턴5_골프
                                    ball_success = False
                                    # delay = 1
                                elif cx_goal_point >= tuned_right_limit:        # hole is at the right side
                                    TX_num = 3                                  # TX3: 오른쪽턴5_골프
                                    ball_success = False
                                    # delay = 1
                                else:
                                    goal_point_success = True
                                    hole_distance = get_hole_distance(hole_width)  # hole 까지의 거리 계산
                                    status = 5
                                    TX_num = 0
                                    # delay = 5
                                    
                        elif status == 5:       # 5: Ball at hit point
                            if TX_num == 0:     # head down
                                head_angle = (0, -80)
                                TX_num = motion_dict[head_angle]
                                delay = 3
                            else:
                                if not ball_detected:
                                    status = 21
                                    # delay = 5
                                else:
                                    if args['map'] == 'par4' and hit_cnt == 0:
                                        limits = [ball_at_hit_point_left_limit_par4, ball_at_hit_point_right_limit_par4, ball_at_hit_point_upper_limit_par4, ball_at_hit_point_lower_limit_par4]
                                    else:
                                        limits = [ball_at_hit_point_left_limit, ball_at_hit_point_right_limit, ball_at_hit_point_upper_limit, ball_at_hit_point_lower_limit]
                                    TX_num = ball_at_hit_point(cx_ball, cy_ball, limits)  # step
                                    # delay = 3
                                    if TX_num == 0 and (not goal_point_success or not ball_success):
                                        ball_success = True
                                        status = 4
                                    elif TX_num == 0 and (goal_point_success and ball_success):
                                        status = 6
                                    else:
                                        ball_success = False
                                        goal_point_success = False
                    
                        elif status == 6:       # 6: Hitting the Ball
                            if TX_num == 0:
                                if hit_direction == 0:  # hit left
                                    if far_shot:
                                        TX_num = 34
                                        hit_strength = 1
                                    elif near_hole_detected:
                                        TX_num = 35     # TX35: 골프_왼쪽으로_샷3
                                        hit_strength = 0
                                    elif hole_distance > 200:
                                        TX_num = 2      # TX2: 골프_왼쪽으로_샷1
                                        hit_strength = 2
                                    elif hole_distance > 130:
                                        TX_num = 34     # TX34: 골프_왼쪽으로_샷2
                                        hit_strength = 1 
                                    else:
                                        TX_num = 35     # TX35: 골프_왼쪽으로_샷3
                                        hit_strength = 0
                                else:
                                    TX_num = 5          # TX5: 골프_오른쪽으로_샷1
                                delay = 5
                            else:
                                TX_num = 0
                                # delay = 5
                                if far_shot:
                                    if hit_strength == 1:
                                        after_hit_move_cnt = after_hit_move1
                                    elif hit_strength == 2:
                                        after_hit_move_cnt = after_hit_move2
                                    else:
                                        after_hit_move_cnt = 0
                                    status = 61
                                else:
                                    status = 7

                        elif status == 61:      # Dash Toward Ball
                            if after_hit_move_cnt > 0:
                                TX_num = 14
                                after_hit_move_cnt -= 1
                            else:
                                status = 7
                                TX_num = 0
                                # delay = 5

                        elif status == 7:       # 7: Tracking Ball
                            if TX_num == 0:
                                if far_shot:
                                    head_angle_x = 90 if hit_direction == 0 else -90
                                    head_angle = (head_angle_x, -30)
                                else:
                                    head_angle_x = 45 if hit_direction == 0 else -45
                                    head_angle = (head_angle_x, -45)
                                TX_num = motion_dict[head_angle]
                                delay = 3
                                hit_cnt += 1
                            else:
                                # elif far_shot and not ball_detected:
                                #     TX_data(serial_port, 23)
                                #     break
                                if ball_detected:
                                    # delay = 5
                                    TX_num = 0
                                    status = 0
                                else:
                                    if head_angle[1] == -45:
                                        status = 0
                                        head_angle = (-0, -30)
                                    else:
                                        head_angle = (head_angle_x, head_angle[1] - 15) # -0도 -> -15도 -> 30도 -> -45도 -> -60도 -> -0도
                                    TX_num = motion_dict[head_angle]
                                    delay = 3


                        print("TX_num: {}".format(TX_num))
                        TX_data(serial_port, TX_num)
                      
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
            # cv2.imshow('Mask0: pink_ball', mask0)
            # cv2.imshow('Mask1: yellow_outer_hole', mask1)
            # cv2.imshow('Mask2: black_inner_hole', mask2)
            # cv2.imshow('Mask3: light_green_field', mask3)
            # cv2.imshow('Mask4: dark_green_boundary', mask4)
            # cv2.imshow('Mask5: yellow_near_hole', mask5)
            # cv2.imshow('Mask6: light_green_field_for_ball', mask6)
            cv2.imshow('Mask7: grey_bynker', mask7)


        key = 0xFF & cv2.waitKey(1)
        
        if key == 27:  # ESC  Key
            TX_data(serial_port, 0)
            break
        elif key == ord(' '):  # spacebar Key
            if View_select == 0:
                View_select = 1
                if  start_time is None:
                    start_time = clock()
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