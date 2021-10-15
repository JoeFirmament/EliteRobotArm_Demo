import PySimpleGUI as sg
import time
import threading
from loguru import logger
import configparser


#--GLOBAL---
FONT_SIZE = 82
MAX_THRESHOLD_DIMENTION = 400  
REFERENCE_SIZE = 200
TYPE_MODE = 0
MAX_POINTS = 8
DIMENSION = {}
MEASURE_TIMES = 0
ACK_TIMES = 0 

# 读取预设文件，获取参数
# MAX_THRESHOLD_DIMENTION 为 轮廓面积的阈值，小于该阈值的不被检测
# TYPE_MODE 检测方式： 0 为 realsense检测方式；1 为单目，像素比/尺寸 的检测方式，需要在左上角放置方形
# FEFERENCE_SIZE 参考物尺寸： 当检测方式为1（单目）时，左上角正方形的真实物理尺寸（以mm为单位）

def readSetting():
    global MAX_POINTS,TYPE_MODE
    cf = configparser.ConfigParser()
    cf.read("./setting.conf")
    MAX_POINTS = int(cf.get("SETTING", "MAX_THRESHOLD_DIMENTION"))
    TYPE_MODE = int(cf.get("SETTING", "TYPE_MODE"))
    REFERENCE_SIZE = int(cf.get("SETTING", "REFERENCE_SIZE"))
    print(MAX_POINTS,TYPE_MODE)


readSetting()



#--Log--
logger.add('logs/monitor.log',
           level='DEBUG',
           format='{time:YYYY-MM-DD HH:mm:ss} - {level} - {file} - {line} - {message}',
           rotation="10 MB")

#--software beginning time--
start = time.time()

#--ADC--
def adcMonitor(window):
    global MAX_POINTS
    global start
    start = time.time() ## probe start time
    while True :
        for i in range(10):
            time.sleep(1)
            if i<MAX_POINTS :
                window.write_event_value('-LED-', i)
            window.write_event_value('-MIN-',i*2)
            window.write_event_value('-HOUR-',i*10)
            window.write_event_value('-DAY-',i*100)
            elaspedTime = time.strftime("%H:%M:%S", time.gmtime(time.time()-start))
            logger.info(elaspedTime)
            window.write_event_value('-TIME-',elaspedTime)
            
            
#--threading
def adcThreading():
    threading.Thread(target=adcMonitor, args=(window,), daemon=True).start()

#--layout--
layout = [
        [sg.T("Measurement Status",font="Helvetica ")],

        [sg.Graph(canvas_size=(900,100),graph_bottom_left=(0,0),graph_top_right=(900,100),background_color="grey96",key='graph')],
        [sg.T("12",size=(6,None),font="Helvetica "  + str(FONT_SIZE), key='min',justification="left"),
         sg.T("555",size=(6,None),font="Helvetica "  + str(FONT_SIZE), key='hour',justification="center"),
         sg.T("3456",size=(7,None),font="Helvetica "  + str(FONT_SIZE), key='day',justification="right")],
        [sg.T("Length",size=(45,None),justification="left"),
         sg.T("Width",size=(50,None),justification="center"),
         sg.T("Area",size=(50,None),justification='right')],
        [sg.Text('_'*152,size=(150,None),justification='center')],           # make a horizontal line stretching 152 characters as separator
        [sg.T("Processing Log",font="Helvetica ")],

        [sg.Output(size=(144,10),key='-OUTPUT-')],
        [sg.Button('Init'),sg.Button('Run'),sg.Button('Stop'),sg.Button('Exit'),
         sg.T('                        '*4,size=(100,None)),
         sg.T('Elapsed time : ',font="Helvetica "),
         sg.Text("0000",size=(10,None),font="Helvetica ", key='time',justification="right")] 
        ]

#-- window--

#window = sg.Window('Sensor Status Monitor',layout,element_justification='CENTER')
window = sg.Window('Dimenstion Detection by MKMH',layout)
window.Finalize()

#--Status LED
graph = window['graph']   
for i in range(MAX_POINTS):
    points[i] =graph.DrawPoint(((int(900/(MAX_POINTS+1))*(i+1),50)),40,color='LightSkyBlue4')   
rectangle = graph.DrawRectangle((0,0), (900,100), line_color='grey88'  )      
     

while True:      
    event, values = window.read()
    print(event,values)      
    if event == sg.WIN_CLOSED or event == 'Exit':      
        break      
    if event == 'Init':      
        graph.TKCanvas.itemconfig(points[1], fill = "Blue")      
    elif event == 'Run': 
        adcThreading()     
        graph.TKCanvas.itemconfig(points[0], fill = "Red")    
    elif event == '-LED-':
        graph.TKCanvas.itemconfig(points[values[event]], fill = "PaleVioletRed1")  
    elif event == '-DAY-':
        window['day'].update(values['-DAY-'])
    elif event == '-MIN-':
        window['min'].update(values['-MIN-'])
    elif event == '-HOUR-':
        window['hour'].update(values['-HOUR-'])
    elif event == '-TIME-':
        window['time'].update(values['-TIME-'])
    elif event == 'Exit':      
        break  
window.close()