import serial
from threading import Timer
from serial.tools import list_ports
import logging

# ser = serial.Serial("COM1", 38400) #打开COM1并设置波特率为115200，COM1只适用于Windows
ser =None
def serial_init(portx,bps,timeout):
    global ser
    try:
        ser = serial.Serial(portx, bps, timeout=timeout)  # winsows系统使用com8口连接串行口
        ser.baudrate(38400)
        # 判断是否打开成功
        if not (ser.is_open()):
            ser.open()
    except Exception as e:
        print("---异常---：", e)

    return ser
    # 打开端口
    # print(ser.name)
    # print(ser.baudrate)  # 波特率
    # print(ser.bytesize)  # 字节大小
    # print(ser.parity)  # 校验位N－无校验，E－偶校验，O－奇校验
    # print(ser.stopbits)  # 停止位
    # print(ser.timeout)  # 读超时设置
    # print(ser.writeTimeout)  # 写超时
    # print(ser.xonxoff)  # 软件流控
    # print(ser.rtscts)  # 硬件流控
    # print(ser.dsrdtr)  # 硬件流控
    # print(ser.interCharTimeout)  # 字符间隔超时

def serial_rec_Data():
    try:
        if ser.in_waiting:
            return ser.readline().decode("gbk")
    except KeyboardInterrupt:
        if ser != None:
            ser.close()

# def serial_sent_Data(data):
#     ser.write(data)

def find_all_devices():
    '''
    线程检测连接设备的状态
    '''
    find_all_serial_devices()
    start_thread_timer(find_all_devices, 1)
        
def start_thread_timer(callback, timer=1):
    '''
    util: start thread timer
    '''
    temp_thread = Timer(timer, callback)
    temp_thread.setDaemon(True)
    temp_thread.start()

def find_all_serial_devices():
    '''
    检查串口设备
    '''
    try:
        serial_listbox = list()
        temp_serial = list()
        for com in list(list_ports.comports()):
            strCom = com[0] + ": " + com[1][:-7]
            temp_serial.append(strCom)
        # for item in temp_serial:
        #     if item not in serial_listbox:
        #         serial_frm.frm_left_listbox.insert("end", item)
        # for item in serial_listbox:
        #     if item not in temp_serial:
        #         size = serial_frm.frm_left_listbox.size()
        #         index = list(serial_frm.frm_left_listbox.get(
        #             0, size)).index(item)
        #         serial_frm.frm_left_listbox.delete(index)

        serial_listbox = temp_serial
        return serial_listbox
    except Exception as e:
        logging.error(e)
