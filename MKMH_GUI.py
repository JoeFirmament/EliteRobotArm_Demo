import PySimpleGUI as sg
import time
import threading
from loguru import logger
import configparser
import cv2
from Q_getIP import getIP
import Q_globals as glo
from Q_calPoint import get_3d_coordinates
from Q_calPoint import order_points
import pyrealsense2 as rs
from scipy.spatial.distance import euclidean
from imutils import perspective
from imutils import contours
import imutils
import numpy as np
import json

def get_aligned_images():
    #logger.info("Getting aligned images")
    frames = pipeline.wait_for_frames()  #等待获取图像帧
    aligned_frames = align.process(frames)  #获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  #获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()   #获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics   #获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  #获取深度参数（像素坐标系转相机坐标系会用到）
    camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }
    # 保存内参到本地
    with open('./intrinsics.json', 'w') as fp:
        json.dump(camera_parameters, fp)
    #######################################################
    
    depth_image = np.asanyarray(aligned_depth_frame.get_data())  #深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  #深度图（8位）
    depth_image_3d = np.dstack((depth_image_8bit,depth_image_8bit,depth_image_8bit))  #3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图
    
    #返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame


def readSetting():
    logger.info("Read Setting file")
    print("Read setting file done.")
    cf = configparser.ConfigParser()
    cf.read("./setting.conf")
    glo.MAX_POINTS = int(cf.get("SETTING", "MAX_THRESHOLD_DIMENTION"))
    glo.TYPE_MODE = int(cf.get("SETTING", "TYPE_MODE"))
    glo.REFERENCE_SIZE = int(cf.get("SETTING", "REFERENCE_SIZE"))
    glo.DISPLAY_STATUS = int(cf.get("SETTING", "DISPLAY_STATUS"))



#--ADC--
def adcMonitor(window):
    global MAX_POINTS
    global start
    start = time.time() ## probe start time
    while True :
        for i in range(10):
            time.sleep(1)
            if i< 8 :
                window.write_event_value('-LED-', i)
            window.write_event_value('-MIN-',i*2)
            window.write_event_value('-HOUR-',i*10)
            window.write_event_value('-DAY-',i*100)
            elaspedTime = time.strftime("%H:%M:%S", time.gmtime(time.time()-start))
            logger.info(elaspedTime)
            window.write_event_value('-TIME-',elaspedTime)
            
            

#--threading
def measureThreading():
    threading.Thread(target=adcMonitor, args=(window,), daemon=True).start()

'''
Main 
'''
#------Init globals--------
glo.init()

start = time.time()
readSetting()
local_IP_Address = getIP()

#-----Init loger
logger.add('logs/monitor.log',
           level='DEBUG',
           format='{time:YYYY-MM-DD HH:mm:ss} - {level} - {file} - {line} - {message}',
           rotation="10 MB")


#-----Hard device init
pipeline = rs.pipeline()  #定义流程pipeline
config = rs.config()   #定义配置config
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  #配置depth流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)   #配置color流
profile = pipeline.start(config)  #流程开始
align_to = rs.stream.color  #与color流对齐
align = rs.align(align_to)
logger.info("Open RealSense Device ")
#logger.info(glo.DISPLAY_STATUS)


#video_capture = cv2.VideoCapture(0)
camera_Width  = 320 # 480 # 640 # 1024 # 1280
camera_Heigth = 240 # 320 # 480 # 780  # 960
frameSize = (camera_Width, camera_Heigth)


#--layout--
colwebcam1_layout = [[sg.Text("Camera View", size=(60, 1), justification="c")],
                        [sg.Image(filename="", key="cam1")]]
colwebcam1 = sg.Column(colwebcam1_layout, element_justification='c')

colwebcam2_layout = [[sg.Text("Measurement View", size=(60, 1), justification="c")],
                        [sg.Image(filename="", key="cam1gray")]]
colwebcam2 = sg.Column(colwebcam2_layout, element_justification='c')
colslayout = [colwebcam1, colwebcam2]


layout = [
        colslayout,
        [sg.Text('_'*120,size=(120,None),justification='center')],           # make a horizontal line stretching 152 characters as separator
        [sg.T("Measurement Status",font="Helvetica ")],
        [sg.T("45.67",size=(6,None),font="Helvetica "  + str(glo.FONT_SIZE), key='length',justification="left"),
         sg.T("74.34",size=(6,None),font="Helvetica "  + str(glo.FONT_SIZE), key='width',justification="center"),
         sg.T("3456.65",size=(7,None),font="Helvetica "  + str(glo.FONT_SIZE), key='area',justification="right")],
        [sg.T("Length(mm)",size=(30,None),justification="left"),
         sg.T("Width(mm)",size=(30,None),justification="center"),
         sg.T("Area(square mm)",size=(30,None),justification='right')],
        [sg.Text('_'*120,size=(120,None),justification='center')],           # make a horizontal line stretching 152 characters as separator
        [sg.T("SETTING",font="Helvetica ")],
        [sg.T("LOCAL_IP_ADDRESS",size=(50,None),justification="right"),
         sg.T(local_IP_Address,size=(50,None),key='my_local_ip_address',justification="right")],
        [sg.T("MAX_THRESHOLD_DIMENTION",size=(50,None),justification="right"),
         sg.T(glo.MAX_THRESHOLD_DIMENTION,size=(50,None),key='my_max_threshold_dimention',justification="right")],
        [sg.T("REFERENCE_SIZE",size=(50,None),justification="right"),
         sg.T(glo.REFERENCE_SIZE,size=(50,None),key='my_reference',justification="right")],
        [sg.T("TYPE_MODE",size=(50,None),justification="right"),
         sg.T(glo.TYPE_MODE,size=(50,None),key='my_type_mode',justification="right")],
        [sg.T("MEASURE_TIMES",size=(50,None),justification="right"),
         sg.T(glo.MEASURE_TIMES,size=(50,None),key='my_measure_times',justification="right")],
        [sg.T("ACK_TIMES",size=(50,None),justification="right"),
         sg.T(glo.ACK_TIMES,size=(50,None),key='my_ack_times',justification="right")],
        [sg.T('ELAPSED TIME : ',size=(50,None),justification="right"),
         sg.Text("0000",size=(50,None),key='time',justification="right")],
        [sg.Text('_'*120,size=(120,None),justification='center')],           
        [sg.Checkbox('DISPLAY',size=(30,None),key='-DISPLAY-'), sg.Checkbox('SERVER_ON',size=(30,None),key='-SERVERON-'),
        sg.Radio('SINGLE CAM','work_mode',size=(30,None),key ='-SINGLECAM-'),sg.Radio('3D CAM','work_mode',default='true',size=(30,None),key = '-3DCAM-')],
        [sg.Text('_'*120,size=(120,None),justification='center')],           
        [sg.T("Processing Log",font="Helvetica ")],
        [sg.Output(size=(120,8),key='-OUTPUT-')],
        [sg.Button('InitDevice'),sg.Button('LoadSetting'),sg.Button('ResetTimmer'),sg.Button('Run'),
        sg.Button('Stop'),sg.Button('Exit')]
         

        ]

#-- window--

window = sg.Window('Dimenstion Detection by MKMH',layout)
window.Finalize() #各个元素是否同步显示

recording = True
while True: 
    elaspedTime = time.strftime("%H:%M:%S", time.gmtime(time.time()-start))
    logger.info(elaspedTime)
    window.write_event_value('-TIME-',elaspedTime) 
        
    event, values = window.read(timeout=25)
    intr, depth_intrin, rgb, depth, aligned_depth_frame = get_aligned_images() #获取对齐的图像与相机内参
    #print(event,values)   
    #print(type(values))   
    print(values['-DISPLAY-'])
    if recording:
    # get camera frame
        frameOrig = rgb
        
    # # update webcam1
        if(glo.DISPLAY_STATUS == 1):
            frame = cv2.resize(frameOrig, frameSize)
            imgbytes = cv2.imencode(".png", frame)[1].tobytes()
            window["cam1"].update(data=imgbytes)
    


    # # transform frame to grayscale
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 0)
        #cv2.imshow("blur", blur)

        #以下为opencv中的形态学处理

        #Canny函数为非微分边缘检测算法，具有滤波、增强、检测的多阶段优化算子。
        edged = cv2.Canny(blur, 50, 100)

        #图像膨胀的用途：
        #用途1：Dilation 影像膨脹通常是配合著影像侵蝕 Erosion 使用，先使用侵蝕的方式使影像中的線條變窄，同時也去除雜訊，之後再透過 Dilation 將影像膨脹回來。
        #用途2：用來連接兩個很靠近但分開的物體。
        edged = cv2.dilate(edged, None, iterations=1) # 图像膨胀


        #图像腐蚀Erosion的用途：
        #用途1：Erosion 影像侵蝕對於移除影像中的小白雜點很有幫助，可用來去噪，例如影像中的小雜點，雜訊。
        #用途2：細化影像，消除毛刺。
        edged = cv2.erode(edged, None, iterations=1) #图像腐蚀
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # Sort contours from left to right as leftmost contour is reference object
        #(cnts, _) = contours.sort_contours(cnts)

        # Remove contours which are not large enough
        # 做一个阈值滤波，如果尺寸小于某个范围的就排除
        cnts = [x for x in cnts if cv2.contourArea(x) > glo.MAX_THRESHOLD_DIMENTION]
        contours2 = sorted(cnts, key=cv2.contourArea, reverse=True)


        if len(contours2) > 1:
            c = contours2[1]
            #c = [x for x in contours2 if cv2.contourArea(x) > 400]
            cv2.drawContours(rgb, c, -1, (0,250,0), 2)

        

        # 画四个边缘点

            rect = order_points(c.reshape(c.shape[0], 2))
        #print(rect)


            xs = [i[0] for i in rect]
            ys = [i[1] for i in rect]
            xs.sort()
            ys.sort()
            #内接矩形的坐标为
            #print(xs,ys)
            #print(xs[1],xs[2],ys[1],ys[2])
            print(type(rect[0]))
            print(rect[0])
            rgb = cv2.circle(rgb,tuple(rect[0]),radius=6, color=(255,0,255),thickness=2)
            rgb = cv2.circle(rgb,tuple(rect[1]),radius=6, color=(255,0,255),thickness=2)
            rgb = cv2.circle(rgb,tuple(rect[2]),radius=6, color=(255,0,255),thickness=2)
            rgb = cv2.circle(rgb,tuple(rect[3]),radius=6, color=(255,0,255),thickness=2)


        # Reference object dimensions
        # Here for reference I have used a 2cm x 2cm square
        # 设置一个参考矩形用作Marker和基础尺寸，算出实际尺寸与像素尺寸比。
        # 左上角放置一个正方形
        if glo.TYPE_MODE == 1 :
            ref_object = cnts[0]
            box = cv2.minAreaRect(ref_object)
            box = cv2.boxPoints(box)
            box = np.array(box, dtype="int")
            box = perspective.order_points(box)
            (tl, tr, br, bl) = box
            dist_in_pixel = euclidean(tl, tr)
            dist_in_cm = 2
            pixel_per_cm = dist_in_pixel/dist_in_cm


        pixel_per_cm = 1
        # 画剩下的轮廓
        for cnt in cnts:
        	#print(cnt[0])
        	#print(cnt[0][0][0])
        	#print(cnt[0][0][1])
            #画白圈
        	leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
        	rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
        	topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
        	bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
        	rgb = cv2.circle(rgb,leftmost,radius=6, color=(255,255,255),thickness=2)
        	rgb = cv2.circle(rgb,rightmost,radius=6, color=(255,255,255),thickness=2)
        	rgb = cv2.circle(rgb,topmost,radius=6, color=(255,255,255),thickness=2)
        	rgb = cv2.circle(rgb,bottommost,radius=6, color=(255,255,255),thickness=2)
        	#image = cv2.circle(image,cnt[0],radius=6, color=(255,255,255),thickness=2)


        	box = cv2.minAreaRect(cnt)
        	box = cv2.boxPoints(box)
        	box = np.array(box, dtype="int")

        	box = perspective.order_points(box)


        	(tl, tr, br, bl) = box
        	cv2.drawContours(rgb, [box.astype("int")], -1, (0, 0, 255), 2)
        	mid_pt_horizontal = (tl[0] + int(abs(tr[0] - tl[0])/2), tl[1] + int(abs(tr[1] - tl[1])/2))
        	mid_pt_verticle = (tr[0] + int(abs(tr[0] - br[0])/2), tr[1] + int(abs(tr[1] - br[1])/2))
        	wid = euclidean(tl, tr)/pixel_per_cm
        	ht = euclidean(tr, br)/pixel_per_cm
        	cv2.putText(rgb, "{:.1f}cm".format(wid), (int(mid_pt_horizontal[0] - 15), int(mid_pt_horizontal[1] - 10)), 
        		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        	cv2.putText(rgb, "{:.1f}cm".format(ht), (int(mid_pt_verticle[0] + 10), int(mid_pt_verticle[1])), 
        		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        # Draw contours
        # 画出边缘
        cv2.drawContours(rgb, cnts, -1, (0,250,0), 2)

    # # update webcam2

        if(glo.DISPLAY_STATUS == 1):
            print(glo.DISPLAY_STATUS)
            frame = cv2.resize(rgb, frameSize)
            imgbytes = cv2.imencode(".png", frame)[1].tobytes()
            window["cam1gray"].update(data=imgbytes)
    #if values['-SINGLECAM-'] == True and values['-3DCAM-'] == False :
    #    glo.TYPE_MODE = 1
    #elif values['-SINGLECAM-'] == False and values['-3DCAM-'] == True :
    #    glo.TYPE_MODE = 0
    if values['-DISPLAY-'] == True:
        glo.DISPLAY_STATUS = 1
    else:
        glo.DISPLAY_STATUS = 0
    if event == sg.WIN_CLOSED or event == 'Exit':      
        break      
    if event == 'LoadSetting':    
        readSetting()
        window['my_max_threshold_dimention'].update(glo.MAX_THRESHOLD_DIMENTION)
        window['my_reference'].update(glo.REFERENCE_SIZE)
        window['my_type_mode'].update(glo.TYPE_MODE)
    elif event == 'ResetTimmer':
        start = time.time()     #    measureThreading()     
    #    graph.TKCanvas.itemconfig(points[0], fill = "Red")    
    #elif event == '-LED-':
    #    graph.TKCanvas.itemconfig(points[values[event]], fill = "PaleVioletRed1")  
    #elif event == '-DAY-':
    #    window['day'].update(values['-DAY-'])
    #elif event == '-MIN-':
    #    window['min'].update(values['-MIN-'])
    #elif event == '-HOUR-':
    #    window['hour'].update(values['-HOUR-'])
    if event == '-TIME-':
        window['time'].update(values['-TIME-'])
    elif event == 'Exit':      
        break  
pipeline.stop()
window.close()