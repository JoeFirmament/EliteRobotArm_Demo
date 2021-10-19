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
    try:
        #logger.info("Getting aligned images")
        frames = pipeline.wait_for_frames()  #等待获取图像帧
        aligned_frames = align.process(frames)  #获取对齐帧
        aligned_depth_frame = aligned_frames.get_depth_frame()  #获取对齐帧中的depth帧
        color_frame = aligned_frames.get_color_frame()   #获取对齐帧中的color帧
        ############### 相机参数的获取 #######################
        intr = color_frame.profile.as_video_stream_profile().intrinsics   #获取相机内参
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  #获取深度参数（像素坐标系转相机坐标系会用到）
        camera_parameters = {'fx': intr.fx, 'fy': intr.fy,'ppx': intr.ppx, 'ppy': intr.ppy,'height': intr.height, 'width': intr.width,'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()}
        # 保存内参到本地
        with open('./intrinsics.json', 'w') as fp:
            json.dump(camera_parameters, fp)
        #######################################################

        depth_image = np.asanyarray(aligned_depth_frame.get_data())  #深度图（默认16位）
        #depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  #深度图（8位）
        #depth_image_3d = np.dstack((depth_image_8bit,depth_image_8bit,depth_image_8bit))  #3通道深度图
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        color_image = np.asanyarray(color_frame.get_data())  # RGB图
    
        #返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
        return intr, depth_intrin, color_image, depth_image, aligned_depth_frame,depth_colormap
    except Exception as e:
        logger.error("Grap Frame Error")


def readSetting():
    try:
        cf = configparser.ConfigParser()
        cf.read("./setting.conf")
        glo.MAX_THRESHOLD_DIMENTION = int(cf.get("SETTING", "MAX_THRESHOLD_DIMENTION"))
        glo.TYPE_MODE = int(cf.get("SETTING", "TYPE_MODE"))
        glo.REFERENCE_SIZE = int(cf.get("SETTING", "REFERENCE_SIZE"))
        glo.DISPLAY_STATUS = int(cf.get("SETTING", "DISPLAY_STATUS"))
    except Exception as e:
        logger.error("ReadSettingFileError",e)
    else :
        logger.info("Read Setting file")
        print("Read setting file done.")   



#--Measurement Func--
def measurement_realsense(window):
    logger.info("Open RealSense Device ")
    frameSize = (320,240)
    while True:
        elaspedTime = time.strftime("%H:%M:%S", time.gmtime(time.time()-start))
        window.write_event_value('-TIME-',elaspedTime)    
        intr, depth_intrin, rgb, depth, aligned_depth_frame,depth_color = get_aligned_images() #获取对齐的图像与相机内参
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (7, 7), 0)       
        edged = cv2.Canny(blur, 50, 100)        
        edged = cv2.dilate(edged, None, iterations=1) # 图像膨胀
        edged = cv2.erode(edged, None, iterations=1) #图像腐蚀
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts = [x for x in cnts if cv2.contourArea(x) > glo.MAX_THRESHOLD_DIMENTION]
        contours2 = sorted(cnts, key=cv2.contourArea, reverse=True) # 获得所有轮廓以包围面积的从大到小排序
        if glo.DISPLAY_STATUS == 1:
            frameOrig = rgb
            frame = cv2.resize(frameOrig, frameSize)
            imgbytes = cv2.imencode(".png", frame)[1].tobytes()
            window["cam1"].update(data=imgbytes)
        if glo.TYPE_MODE == 0:
            if len(contours2) == 1 :
                logger.info("Work on 3D CAM Mode ,Get 1 contour")
                objContour = contours2[0]
                rect = order_points(objContour.reshape(objContour.shape[0], 2))
                box = cv2.minAreaRect(objContour)
                box = cv2.boxPoints(box)
                box = np.array(box, dtype="int")
                box = perspective.order_points(box)
                (tl, tr, br, bl) = box
                #print(tl)
                #print(type(tl))
                width = get_3d_coordinates(intr,aligned_depth_frame,tl[0],tl[1],tr[0],tr[1])*1000
                length = get_3d_coordinates(intr,aligned_depth_frame,tl[0],tl[1],bl[0],bl[1])*1000
                width = round(width,2)
                length = round(length,2)
                window.write_event_value('-WIDTH-',width) 
                window.write_event_value('-LENGTH-',length) 

                   

                print(length)
                if glo.DISPLAY_STATUS == 1:
                    depth_color = cv2.circle(depth_color,tuple(rect[0]),radius=6, color=(255,0,255),thickness=2)
                    depth_color = cv2.circle(depth_color,tuple(rect[1]),radius=6, color=(255,0,255),thickness=2)
                    depth_color = cv2.circle(depth_color,tuple(rect[2]),radius=6, color=(255,0,255),thickness=2)
                    depth_color = cv2.circle(depth_color,tuple(rect[3]),radius=6, color=(255,0,255),thickness=2)
                    frame = cv2.resize(depth_color, frameSize)
                    imgbytes = cv2.imencode(".png", frame)[1].tobytes()
                    window["cam1gray"].update(data=imgbytes)

            else:
                print("Too much contours ,pls keep view clear")
        if glo.TYPE_MODE == 1:
            if len(contours2) > 1:
                logger.info("SingleCam Mode")
                objContour = contours2[0]
                refContour = contours2[1]

                rect = order_points(refContour.reshape(refContour.shape[0], 2))
                box = cv2.minAreaRect(refContour)
                box = cv2.boxPoints(box)
                box = np.array(box, dtype="int")
                box = perspective.order_points(box)
                (tl, tr, br, bl) = box
                dist_in_pixel = euclidean(tl, tr)
                dist_in_mm = glo.REFERENCE_SIZE 
                pixel_per_mm = dist_in_pixel/dist_in_mm #获得实尺寸和像素之间的比例关系
                # 把所有的轮廓全描出来，算一遍
                for cnt in cnts:
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
        	        wid = euclidean(tl, tr)/pixel_per_mm
        	        ht = euclidean(tr, br)/pixel_per_mm
        	        cv2.putText(rgb, "{:.1f}mm".format(wid), (int(mid_pt_horizontal[0] - 15), int(mid_pt_horizontal[1] - 10)), 
        	        	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        	        cv2.putText(rgb, "{:.1f}mm".format(ht), (int(mid_pt_verticle[0] + 10), int(mid_pt_verticle[1])), 
        		    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    # Draw contours
                    # 画出边缘
                cv2.drawContours(rgb, cnts, -1, (0,250,0), 2)

                if glo.DISPLAY_STATUS == 1:
                    rgb = cv2.circle(rgb,tuple(rect[0]),radius=6, color=(255,0,255),thickness=2)
                    rgb = cv2.circle(rgb,tuple(rect[1]),radius=6, color=(255,0,255),thickness=2)
                    rgb = cv2.circle(rgb,tuple(rect[2]),radius=6, color=(255,0,255),thickness=2)
                    rgb = cv2.circle(rgb,tuple(rect[3]),radius=6, color=(255,0,255),thickness=2)
                    frame = cv2.resize(rgb, frameSize)
                    imgbytes = cv2.imencode(".png", frame)[1].tobytes()
                    window["cam1gray"].update(data=imgbytes)
            #time.sleep(1/50)

#--threading
def measureThreading():
    global window
    logger.info("Measuring")
    threading.Thread(target=measurement_realsense, args=(window,), daemon=True).start()



#------Init globals--------
glo.init()
start = time.time() # 为判断GUI线程是否挂掉
readSetting()
local_IP_Address = getIP() #只取C类地址，以192.168开头
#-----Init loger
logger.add('logs/monitor.log',level='DEBUG',format='{time:YYYY-MM-DD HH:mm:ss} - {level} - {file} - {line} - {message}',rotation="10 MB")
#-----Hard device init
pipeline = rs.pipeline()  #定义流程pipeline
config = rs.config()   #定义配置config
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  #配置depth流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)   #配置color流
profile = pipeline.start(config)  #流程开始

align_to = rs.stream.color  #与color流对齐
align = rs.align(align_to)

colorizer = rs.colorizer()


### -----layout
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
        [sg.Checkbox('DISPLAY',size=(30,None),default=True,key='-DISPLAY-'), sg.Checkbox('SERVER_ON',size=(30,None),key='-SERVERON-'),
        sg.Radio('SINGLE CAM','work_mode',size=(30,None),key ='-SINGLECAM-'),sg.Radio('3D CAM','work_mode',default='true',size=(30,None),key = '-3DCAM-')],
        [sg.Text('_'*120,size=(120,None),justification='center')],           
        [sg.T("Processing Log",font="Helvetica ")],
        #  [sg.Output(size=(120,8),key='-OUTPUT-')],       # 这个控件接管了print
        [sg.Button('LoadSetting'),sg.Button('ResetTimmer'),sg.Button('Run'),sg.Button('ClearOutput'),sg.Button('Exit')]
        ]

window = sg.Window('Dimenstion Detection by MKMH',layout)
#window.Finalize() #各个元素是否同步显示
while True: 
    #elaspedTime = time.strftime("%H:%M:%S", time.gmtime(time.time()-start))
    #logger.info(elaspedTime)
    #window.write_event_value('-TIME-',elaspedTime)     
    event, values = window.read(timeout=100)   
    if values['-SINGLECAM-'] == True and values['-3DCAM-'] == False :
        glo.TYPE_MODE = 1
    elif values['-SINGLECAM-'] == False and values['-3DCAM-'] == True :
        glo.TYPE_MODE = 0
    if values['-DISPLAY-'] == True:
        glo.DISPLAY_STATUS = 1
    else:
        glo.DISPLAY_STATUS = 0
    if event == 'Run':
        measureThreading()
    if event == 'LoadSetting':    
        readSetting()
        window['my_max_threshold_dimention'].update(glo.MAX_THRESHOLD_DIMENTION)
        window['my_reference'].update(glo.REFERENCE_SIZE)
        window['my_type_mode'].update(glo.TYPE_MODE)
    elif event == 'ResetTimmer':
        start = time.time()    
#       elif event == 'ClearOutput':
#            window['-OUTPUT-'].update('') 
    if event == sg.WIN_CLOSED or event == 'Exit':      
        break      
    if event == '-TIME-':
        window['time'].update(values['-TIME-'])
    if event == '-WIDTH-':
        window['width'].update(values['-WIDTH-'])
    if event == '-LENGTH-':
        window['length'].update(values['-LENGTH-'])       
pipeline.stop
window.close()
