from myserial import *
from frame import *
from time import time,sleep
from threading import Thread
from queue import Queue
from tkinter import Button,Text,Listbox,END,Tk
from matplotlib import use as mpluser
def get_time():
    return time()

def getvalue():
    while True:
        serial_Data = serial_rec_Data()
        if serial_Data != None:
            drive = serial_Data[:2]
            if drive == '01':
                queueset(serial_Data, 2, 4, humidity)
                queueset(serial_Data, 5, 0, temp)
            elif drive == '02':
                queueset(serial_Data, 2, 0, humenboey)
            elif drive == '03':
                queueset(serial_Data, 2, 0, ultrasonic)
            elif drive == '04':
                queueset(serial_Data, 2, 0, photoresistance)
            elif drive == '05':
                queueset(serial_Data, 2, 4, acceleration_x)
                queueset(serial_Data, 5, 7, acceleration_y)
                queueset(serial_Data, 8, 10, acceleration_z)
            else :
                mtext.insert(END,serial_Data)
                mtext.see(END)
                # print(serial_Data)
                continue
def queueset(str, start, end, queue):
    queue.join()
    if queue.full():
        queue.queue.clear()
    if end == 0:
        queue.put(int(str[start:]))
    else:
        queue.put(int(str[start:end]))
    queue.task_done()

def check_init():
    global first_init
    if first_init:
        plt_init()
        start_time = get_time()
        first_init =False


def SerialAll():
    check_init()
    while True:
        pltstart()
        t_now = get_time() - start_time
        ax.append(t_now)
        ay.append(humidity.get())
        bx.append(t_now)
        by.append(temp.get())
        cx.append(t_now)
        cy.append(humenboey.get())
        dx.append(t_now)
        dy.append(ultrasonic.get())
        ex.append(t_now)
        ey.append(photoresistance.get())
        fx.append(t_now)
        fy.append(acceleration_x.get())
        fy1.append(acceleration_y.get())
        fy2.append(acceleration_z.get())
        pltset(1, "湿度记录表", "湿度", ax, ay)
        pltset(2, "温度记录表", "温度°C", bx, by)
        pltset(3, "人体红外记录表", "有人", cx, cy)
        pltset(4, "超声波测距记录表", "距离cm", dx, dy)
        pltset(5, "光敏电阻记录表", "电阻", ex, ey)
        pltset(6, "三轴加速度记录表", "g", fx, fy,fy1,fy2)

        pltpause()


def SerialHumidity():
    check_init()
    while True:
        pltstart()
        t_now = get_time() - start_time
        ax.append(t_now)
        ay.append(humidity.get())
        pltset(0,"湿度记录表", "湿度", ax, ay)
        pltpause()


def SerialTemp():
    check_init()
    while True:
        pltstart()
        t_now = get_time() - start_time
        bx.append(t_now)
        by.append(temp.get())
        pltset(0,"温度记录表", "温度", bx, by)
        pltpause()
def SerialHumenboey():
    check_init()
    while True:
        pltstart()
        t_now = get_time() - start_time
        cx.append(t_now)
        cy.append(humenboey.get())
        pltset(0,"人体红外记录表", "有人", cx, cy)
        pltpause()

def SerialUltrasonic():
    check_init()
    while True:
        pltstart()
        t_now = get_time() - start_time
        dx.append(t_now)
        dy.append(ultrasonic.get())
        pltset(0,"超声波测距记录表", "距离", dx, dy)
        pltpause()

def SerialPhotoresistance():
    check_init()
    while True:
        pltstart()
        t_now = get_time() - start_time
        ex.append(t_now)
        ey.append(photoresistance.get())
        pltset(0,"光敏电阻记录表", "电阻", ex, ey)
        pltpause()
def Serialxyx():
    check_init()
    while True:
        pltstart()
        t_now = get_time() - start_time
        fx.append(t_now)
        fy.append(acceleration_x.get())
        fy1.append(acceleration_y.get())
        fy2.append(acceleration_z.get())

        pltset(7,"三轴加速度记录表", "g", fx, fy,fy1,fy2)
        pltpause()
        
def serial_toggle():
    '''
    打开/关闭串口设备
    '''
    serial_index = mlistbox.curselection()
    if serial_index:
        current_serial_str = mlistbox.get(serial_index)
        port = current_serial_str.split(":")[0]
        serial_init(port, 38400, 1)
        sleep(1)
        t = Thread(target=getvalue)
        t.start()

first_init =True
queuelen = 10
humidity =Queue(queuelen)
temp =Queue(queuelen)
humenboey =Queue(queuelen)
ultrasonic =Queue(queuelen)
photoresistance =Queue(queuelen)
acceleration_x =Queue(queuelen)
acceleration_y =Queue(queuelen)
acceleration_z =Queue(queuelen)

ax,bx,cx,dx,ex,fx =[0],[0],[0],[0],[0],[0]
ay,by,cy,dy,ey,fy,fy1,fy2 =[0],[0],[0],[0],[0],[0],[0],[0]
top = Tk()
top.geometry("300x450")  # tkinter界面长宽，根据像素点来的，每个不同的分辨率电脑上显示不一样
top.title("监控")  # 设置tkinter界面名字

mpluser('TkAgg')
devices = find_all_serial_devices()

mlistbox = Listbox(top,height=3)
for item in devices:
    mlistbox.insert("end", item)
mlistbox.pack()
Button(top, text = '打开串口',command = serial_toggle).pack()
Button(top, text = '全部记录表',command = SerialAll).pack()
Button(top, text = '湿度记录表',command = SerialHumidity).pack()
Button(top, text = '温度记录表',command = SerialTemp).pack()
Button(top, text = '人体红外记录表',command = SerialHumenboey).pack()
Button(top, text = '超声波测距记录表',command = SerialUltrasonic).pack()
Button(top, text = '光敏电阻记录表',command = SerialPhotoresistance).pack()
Button(top, text = '三轴加速度记录表',command = Serialxyx).pack()

mtext = Text(top)
mtext.pack()
start_time = get_time()
top.mainloop()
