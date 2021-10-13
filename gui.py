from turtle import title
import PySimpleGUI as sg      

sg.ChangeLookAndFeel('DarkBlue1')      
 
layout = [      
    [sg.Frame(layout=[  
    [sg.Text("机械臂IP地址  "),sg.InputText("192.168.1.102")],    
    [sg.Text("机械臂运行状态 "),sg.InputText("机械臂正在执行JBI文件")],
    [sg.Text("JBI文件      "),sg.InputText("PICK_SORT.jbi")],
    [sg.Text("工作运行时间      "),sg.InputText("87653Sec")]], title='机械臂信息显示',title_color='yellow', relief=sg.RELIEF_SUNKEN, tooltip='初始化信息',size=(100,10))],   

    [sg.Text('_'  *100)],   

    [sg.Frame(layout=[      
    [sg.Text("           "),sg.Text("传感器#1"),sg.Text("传感器#2"),sg.Text("传感器#3")],
    [sg.Text("串口对应设备名 "),sg.InputText("传感器#1"),sg.InputText("传感器#2"),sg.InputText("传感器#3")],
    [sg.Text("无料时测量值 "),sg.InputText("传感器#1"),sg.InputText("传感器#2"),sg.InputText("传感器#3")],
    [sg.Text("实时测量原始数据 "),sg.InputText("传感器#1"),sg.InputText("传感器#2"),sg.InputText("传感器#3")],
    [sg.Text("单次增量 "),sg.InputText("传感器#1"),sg.InputText("传感器#2"),sg.InputText("传感器#3")],
    [sg.Text("总增量 "),sg.InputText("传感器#1"),sg.InputText("传感器#2"),sg.InputText("传感器#3")]], title='堆垛信息显示',title_color='yellow', relief=sg.RELIEF_SUNKEN, tooltip='堆垛相关信息')],  
    
    [sg.Frame(layout=[
    [sg.Slider(range=(1, 120), orientation='v', size=(5, 20), default_value=25.7),      
     sg.Slider(range=(1, 120), orientation='v', size=(5, 20), default_value=75.5),      
     sg.Slider(range=(1, 120), orientation='v', size=(5, 20), default_value=10.2)]],title='高度示意图',title_color='yellow')],
    [sg.Text('_'  * 100)],   

    [sg.Frame(layout=[  
    [sg.Text("机械臂IP地址  "),sg.InputText("192.168.1.102")],    
    [sg.Text("JBI文件      "),sg.InputText("PICK_SORT.jbi")],
    [sg.Text("1号堆垛对应设备名："),sg.Combo(['COM0','COM1','COM2'])],
    [sg.Text("2号堆垛对应设备名："),sg.Combo(['COM0','COM1','COM2'])],
    [sg.Text("号堆垛对应设备名："),sg.Combo(['COM0','COM1','COM2'])],
    ], title='设置部分',title_color='yellow', relief=sg.RELIEF_SUNKEN, tooltip='设置部分',size=(100,10))],   
    [sg.Output( size=(100, 10))],   

    
]      
window = sg.Window('美客美换分拣控制软件Ver1.0', layout, grab_anywhere=True)      

event, values = window.read()      

window.close()    

