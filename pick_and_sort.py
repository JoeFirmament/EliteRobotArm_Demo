import decimal
from pathlib import Path
from decimal import Decimal
import serial
import re
import time
import logging
import _thread
import elite_json as eli
import PySimpleGUI as sg
'''
定义函数，用于创建线程
'''
def getDistance(serialObj,threadName,delaySec):
    logger.info("测距传感器线程已建立")
    while(1):
        global currentMeasurement
        global distanceMax,distanceMin
        str=serialObj.readline().decode("gbk")
        if len(str) > 13 :
        #print("Thread: ",threadName,":|", str,"|")
            str = re.search(r'[0-9]+(.[0-9]{1,3})?',str,0).group()
            str = Decimal(str)*1000
            # print(str)
            if(Decimal(str)>distanceMin and Decimal(str)<=Decimal(distanceMax)):
                currentMeasurement = str
                # print("测距传感器当前读数： ",currentMeasurement)
            else:
                logger.warning("激光测距读数位于异常范围内")
        else:
            logger.info("传感器此次未读到数据")   
        time.sleep(delaySec)


def robotControl(sock,threadName,delaySec):

    '''
    思路：夹起布草后，现让布草底边接触离机器人较远的台面，后移动到目标程序点，让布草停止晃动，再落下
    P0:第一个程序点（只读）
    P1~5:初始程序点（只读）
    P6～10:目标程序点，（z轴值=布草堆高度+初始程序点z轴值）
    '''
    global currentMeasurement
    global lastMeasurement 
    global increment
    
    logger.info("机器臂控制线程已建立")

    while(1):
            suc, state, id = eli.sendCMD(sock, "getJbiState", {"addr": 1})
            if state['runState'] != 0 or currentMeasurement == 0:
                eli.time.sleep(1.0)
                continue

            print("new currentMeasurement:",currentMeasurement)
            print("lastMeasurement:",lastMeasurement)
            if(Decimal(lastMeasurement) == 0):
                lastMeasurement = currentMeasurement
            inc = Decimal(lastMeasurement)-Decimal(currentMeasurement)
            if abs(inc)>40 :
                #logger.warning("单次增量大于10MM")
                continue
            increment += inc
            print("increment:",increment)

            lastMeasurement = currentMeasurement
            
            # 获取初始程序点各轴角度值
            suc, pos_origin0, id = eli.sendCMD(sock, "getSysVarP", {"addr": 0})
            suc, pos_origin1, id = eli.sendCMD(sock, "getSysVarP", {"addr": 1})
            suc, pos_origin2, id = eli.sendCMD(sock, "getSysVarP", {"addr": 2})
            suc, pos_origin14, id = eli.sendCMD(sock, "getSysVarP", {"addr": 14})
            suc, pos_origin15, id = eli.sendCMD(sock, "getSysVarP", {"addr": 15})
            suc, pos_origin16, id = eli.sendCMD(sock, "getSysVarP", {"addr": 16})
            suc, pos_origin3, id = eli.sendCMD(sock, "getSysVarP", {"addr": 3})
            suc, pos_origin4, id = eli.sendCMD(sock, "getSysVarP", {"addr": 4})

            # 正解（转化为直角坐标值）
            suc, pose_origin0, id = eli.sendCMD(sock, "positiveKinematic", {"targetPos": pos_origin0})
            suc, pose_origin1, id = eli.sendCMD(sock, "positiveKinematic", {"targetPos": pos_origin1})
            suc, pose_origin2, id = eli.sendCMD(sock, "positiveKinematic", {"targetPos": pos_origin2})
            suc, pose_origin14, id = eli.sendCMD(sock, "positiveKinematic", {"targetPos": pos_origin14})
            suc, pose_origin15, id = eli.sendCMD(sock, "positiveKinematic", {"targetPos": pos_origin15})
            suc, pose_origin16, id = eli.sendCMD(sock, "positiveKinematic", {"targetPos": pos_origin16})
            suc, pose_origin3, id = eli.sendCMD(sock, "positiveKinematic", {"targetPos": pos_origin3})
            suc, pose_origin4, id = eli.sendCMD(sock, "positiveKinematic", {"targetPos": pos_origin4})
            
            # 设置目标程序点直角坐标值
            pose_target0 = pose_origin0
            # pose_target0[2] += float(increment)
            pose_target1 = pose_origin1
            pose_target1[2] += float(increment)
            pose_target2 = pose_origin2
            pose_target2[2] += float(increment)
            pose_target14 = pose_origin14
            pose_target14[2] += float(increment)
            pose_target15 = pose_origin15
            pose_target15[2] += float(increment)
            pose_target16 = pose_origin16
            pose_target16[2] += float(increment)
            pose_target3 = pose_origin3
            pose_target3[2] += float(increment)
            pose_target4 = pose_origin4
            pose_target4[2] += float(increment)

            # 逆解（转化为各轴角度值）
            suc, pose_target0, id = eli.sendCMD(sock, "inverseKinematic",{"targetPose": pose_target0, "referencePos": pos_origin0})
            suc, pose_target1, id = eli.sendCMD(sock, "inverseKinematic",{"targetPose": pose_target1, "referencePos": pos_origin1})
            suc, pose_target2, id = eli.sendCMD(sock, "inverseKinematic",{"targetPose": pose_target2, "referencePos": pos_origin2})
            suc, pose_target14, id = eli.sendCMD(sock, "inverseKinematic",{"targetPose": pose_target14, "referencePos": pos_origin14})
            suc, pose_target15, id = eli.sendCMD(sock, "inverseKinematic",{"targetPose": pose_target15, "referencePos": pos_origin15})
            suc, pose_target16, id = eli.sendCMD(sock, "inverseKinematic",{"targetPose": pose_target16, "referencePos": pos_origin16})
            suc, pose_target3, id = eli.sendCMD(sock, "inverseKinematic",{"targetPose": pose_target3, "referencePos": pos_origin3})
            suc, pose_target4, id = eli.sendCMD(sock, "inverseKinematic",{"targetPose": pose_target4, "referencePos": pos_origin4})

            # 设置P5～10
            suc, pose_target0, id = eli.sendCMD(sock, "setSysVarP", {"addr": 5, "pos": pose_target0})
            suc, pose_target1, id = eli.sendCMD(sock, "setSysVarP", {"addr": 6, "pos": pose_target1})
            suc, pose_target2, id = eli.sendCMD(sock, "setSysVarP", {"addr": 7, "pos": pose_target2})
            suc, pose_target14, id = eli.sendCMD(sock, "setSysVarP", {"addr": 40, "pos": pose_target14})
            suc, pose_target15, id = eli.sendCMD(sock, "setSysVarP", {"addr": 41, "pos": pose_target15})
            suc, pose_target16, id = eli.sendCMD(sock, "setSysVarP", {"addr": 42, "pos": pose_target16})
            suc, pose_target3, id = eli.sendCMD(sock, "setSysVarP", {"addr": 8, "pos": pose_target3})
            suc, pose_target4, id = eli.sendCMD(sock, "setSysVarP", {"addr": 9, "pos": pose_target4})

            # 执行脚本
            
            suc, result, id = eli.sendCMD(sock, "runJbi", {"filename": "pick_and_place"})
            logger.info("机械臂开始执行JBI文件")
            time.sleep(delaySec)



#----初始化部分

'''
声明变量
ser=Serial Port 代表串口，Init函数传入getDistance函数
currentMeasurement 产生的激光传感器测量值。
errNo 全局错误码，暂时不用
'''
currentMeasurement = 0
lastMeasurement = 0
increment = 0

#激光测距传感器测量值的范围
distanceMin = 80
distanceMax = 1200




#设置Log输出格式

logging.basicConfig(level = logging.INFO,format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

#logger.info("Start print log")
#logger.debug("Do something")
#logger.warning("Something maybe fail.")
#logger.info("Finish")



#----连接串口
try:
    ser=serial.Serial("/dev/tty.usbserial-14230",38400, timeout=0.5)
    logger.info("串口打开")
except:
    logger.error("串口打开失败")



#----建立与机械臂的socket连接

ip = "192.168.101.200"
try:
    conSuc, sock = eli.connectETController(ip)
    logger.info("与机械臂建立SOCKET连接")
except:
    logger.error("与机器臂socket连接失败")
#----建立线程

_thread.start_new_thread(robotControl,(sock,"robotControl",0.1))

_thread.start_new_thread(getDistance(ser,"getDistance",0.1))


#except:

 #   logger.error("无法启动线程")





    

