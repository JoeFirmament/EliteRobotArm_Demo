def init():
#--GLOBAL---
    global FONT_SIZE,MAX_THRESHOLD_DIMENTION,REFERENCE_SIZE
    global TYPE_MODE,MEASURE_TIMES,ACK_TIMES,SERVER_STATUS,DISPLAY_STATUS
#字体尺寸
    FONT_SIZE = 50

#轮廓尺寸阈值，小于该阈值的，过滤掉
    MAX_THRESHOLD_DIMENTION = 2000 

#参考物的物理实际尺寸
    REFERENCE_SIZE = 200

#工作模式 0 代表3D测量模式， 1 代表 单目相机测量模式
    TYPE_MODE = 0   

#测量次数
    MEASURE_TIMES = 0

#上位机请求次数
    ACK_TIMES = 0 

#发送数据服务器状态
    SERVER_STATUS = 0

#显示状态
    DISPLAY_STATUS = 1

