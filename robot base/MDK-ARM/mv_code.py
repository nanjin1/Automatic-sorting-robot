# Untitled - By: 18314 - 周三 11月 9 2022

import sensor
import image
import time
import struct
from pyb import UART

# 不同区域的的颜色阈值不同
red_threshold = [(0, 66, 36, 96, 96, -92)]
yellow_threshold = [(53, 100, -54, -11, 127, 61)]
blue_threshold = [(30, 82, -23, 121, -94, -9)]
ThresholdList = [red_threshold, yellow_threshold, blue_threshold]  # 圆盘机
# 阶梯平台物块
StaritRedThreshold = [(6, 100, 20, -102, -18, 106)]  # 红色需进行反转
StariBlueThreshold = [(0, 94, 127, -128, -38, 127)]  # 蓝色无需反转
StairThresholdList = [StaritRedThreshold, StariBlueThreshold]
ColorCode = ['red', 'yellow', 'blue']
# 通讯设置
uart = None
uart2 = None
CommunicationDict = {'Size': 5, 'head': 0XFF,
                     'tail': 0x99, 'QR_Head': 0X11, 'QR_Tail': 0X22}
TargetDict = {'Switch': False, 'Target': 0, 'Mode': 0, 'Result': 0X66}
UartDict = {'Lock': True, 'SendTimes': 0, 'if_Uart2': False}
# ROI设定
RegionLimit = [0, 0, 0, 0]  # 初始值
RectangleArea = [0, 0, 0, 0]
CenterList = []
# 调试变量
DebugSwitchDict = {'ShowInfo': True, 'ShowRectangle': True}


# 初始化函数
def CameraInit(framesize=sensor.QVGA):
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(framesize)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)  # 关闭白平衡。
    sensor.set_auto_whitebal(False)
    GetRegionLimit(sensor.snapshot().lens_corr(1.8))


def Uart_Init(uart_id=1, baud_rate=115200, uart_id2=None, baud_rateCopy=True, baud_rate2=115200):
    global uart, uart2, UartDict
    uart = UART(uart_id, baud_rate)
    uart.init(baud_rate, bits=8, parity=None, stop=1)
    if uart_id2 is not None:  # 需要开启两个串口 这二维码支持
        UartDict['if_Uart2'] = True
        if baud_rateCopy:
            baud_rate2 = baud_rate
        uart2 = UART(uart_id2, baud_rate2)
        uart2.init(baud_rate2, bits=8, parity=None, stop=1)
        print(f'串口号{uart_id2}\n'+f'波特率{baud_rate2}\n')
    else:
        UartDict['if_Uart2'] = False


def ClockInit():
    global clock
    clock = time.clock()


def GloablInit(frameSize=sensor.QVGA, uart_id=1, baud_rate=115200, uart_id2=None, baud_rateCopy=True,
               baud_rate2=115200):
    CameraInit(frameSize)
    Uart_Init(uart_id, baud_rate, uart_id2, baud_rateCopy, baud_rate2)
    ClockInit()


# 通讯函数
def uart_receive(i=None):  # mv的数据接收
    if uart.any():
        rec = uart.read()
        # print(rec)
        if len(rec) == CommunicationDict['Size']:
            uart_decode(rec)
    if UartDict['if_Uart2']:
        if uart2.any():
            rec = uart2.read()  # 只做单纯的转发 包装其头尾
            buffer_size = len(rec)
            print(f'数据内容{rec}\n'+f'数据长度{buffer_size}')
            cmd = struct.pack('<' + 'B' * (buffer_size + 1),
                              CommunicationDict['QR_Head'],
                              *rec
                              )
            uart.write(cmd)  # 通过主控的接口进行转发
            print(f'发送指令为：{cmd}')


def uart_decode(rec):  # 对收到的数据进行解码
    global TargetDict, CommunicationDict
    frame_head = rec[0]  # 首位
    frame_tail = rec[CommunicationDict['Size'] - 1]  # 末位
    if (frame_head == CommunicationDict['head'] and
            frame_tail == CommunicationDict['tail'] and
            rec[1] + rec[2] == rec[3]):
        event_id = rec[1]  # 事件ID
        event_param = rec[2]  # 参数值

        if event_id == 0X01:
            TargetDict['Switch'] = True
            TargetDict['Result'] = 0X66
            print("全局开关使能")
            return
        elif event_id == 0X00:
            TargetDict['Switch'] = False
            TargetDict['Result'] = 0X66
            print("全局开关关闭")
            return
        elif event_id == 0X09:
            print("测试数据回传")
            UartDict['Lock'] = False  # 解锁 允许数据回传
            UartDict['SendTimes'] = 4  # 读取后面的4帧图像
            return
        if TargetDict['Switch']:  # 开关此时是打开的
            if event_id == 0X02:  # 圆盘机部分
                TargetDict['Mode'] = 0X02
                print(f"圆盘机 设置目标颜色为{ColorCode[event_param]}")
                TargetDict['Target'] = event_param
            elif event_id == 0X06:  # 阶梯
                TargetDict['Mode'] = 0X06
                print(f"阶梯平台 设置目标颜色为{ColorCode[event_param]}")
                TargetDict['Target'] = event_param


def Uart_Reply():  # openmv发送数据
    global TargetDict, uart
    if not UartDict['Lock']:
        if UartDict['SendTimes'] > 0:
            UartDict['SendTimes'] = UartDict['SendTimes'] - 1  # 自减
            if TargetDict['Result'] != 0X66:  # 识别成功
                # print("发送数据")
                cmd = struct.pack('<BBBBB',
                                  CommunicationDict['head'],
                                  TargetDict['Mode'],
                                  TargetDict['Result'],
                                  (TargetDict['Mode'] + TargetDict['Result']),
                                  CommunicationDict['tail'])
                uart.write(cmd)
            if UartDict['SendTimes'] == 0:
                UartDict['Lock'] = True  # 锁住 关闭发送


# ROI划定函数
def GetRegionLimit(img):  # 基于样例图片
    global RegionLimit, RectangleArea, CenterList
    h_length = img.height()
    w_length = img.width()
    print(h_length)
    print(w_length)
    RegionLimit[0] = float(w_length * 0.3)
    RegionLimit[1] = float(h_length * 0.15)
    RegionLimit[2] = float(w_length * 0.85)
    RegionLimit[3] = float(h_length * 0.95)
    # 读取点位后将其转换为ROI格式
    RectangleArea = [int(RegionLimit[0]),
                     int(RegionLimit[1]),
                     int(RegionLimit[2] - RegionLimit[0]),
                     int(RegionLimit[3] - RegionLimit[1])
                     ]
    CenterList = [int(RectangleArea[0] + RectangleArea[2] * 0.25),
                  int(RectangleArea[1] + RectangleArea[3] * 0.25),
                  int(RectangleArea[2] * 0.7),
                  int(RectangleArea[3] * 0.25)
                  ]
    print(RegionLimit)


def DrawRegion(img):  # 划定ROI
    global RectangleArea, CenterList
    if DebugSwitchDict['ShowRectangle']:  # 调试开关
        thicknessVal = RectangleArea[3] * 0.05
        img.draw_rectangle(RectangleArea[0],
                           RectangleArea[1],
                           RectangleArea[2],
                           RectangleArea[3],
                           color=(255, 0, 0),
                           thickness=int(thicknessVal)
                           )
        img.draw_rectangle(CenterList[0],
                           CenterList[1],
                           CenterList[2],
                           CenterList[3],
                           color=(0, 255, 0),
                           thickness=int(thicknessVal * 0.2)
                           )


# 识别函数 1-非极大值抑制
def find_max_blob(blobs):  # 找到最大色块的函数
    if not blobs:
        return 0
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob


def find_max_SizeParam(blobs):  # 返回最大色块的尺寸
    if not blobs:
        return 0
    max_size = 0
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_size
    # 识别函数 2 - 色块识别


# 识别函数 2-圆盘机色块颜色识别
def Find_TargetBlob(img, threshold,  # 寻找目标色块 圆盘机区域使用
                    roi=None,
                    x_stride=40,
                    y_stride=40,
                    area_threshold=500,
                    pixels_threshold=150,
                    merge=True):
    global TargetDict
    if roi is None:
        blobs = img.find_blobs(threshold,
                               x_stride=x_stride,
                               y_stride=y_stride,
                               area_threshold=area_threshold,
                               pixels_threshold=pixels_threshold,
                               merge=merge)
    else:
        blobs = img.find_blobs(threshold,
                               roi=roi,
                               x_stride=x_stride,
                               y_stride=y_stride,
                               area_threshold=area_threshold,
                               pixels_threshold=pixels_threshold,
                               merge=merge)
    max_blob = find_max_blob(blobs)
    if max_blob:
        if (RegionLimit[0] < max_blob.cx() < RegionLimit[2] and
                RegionLimit[1] < max_blob.cy() < RegionLimit[3]):
            if DebugSwitchDict['ShowInfo']:
                img.draw_string(max_blob.x(), max_blob.y(),
                                f'{max_blob.roundness():.2f}\n',
                                scale=2.0
                                )
                img.draw_rectangle(max_blob.rect())
                img.draw_cross(max_blob.cx(), max_blob.cy())
            TargetDict['Result'] = TargetDict['Target']
        else:
            TargetDict['Result'] = 0X66
    else:
        TargetDict['Result'] = 0X66
    Uart_Reply()


# 识别函数 3- 阶梯平台
def StairFIndTarget(img, threshold,  # 寻找目标色块 圆盘机区域使用
                    if_invert=False,  # 阶梯红色需要进行阈值的反转
                    roi=None,
                    x_stride=40,
                    y_stride=40,
                    area_threshold=500,
                    pixels_threshold=150,
                    merge=True):
    global TargetDict, DebugSwitchDict, CenterList
    if roi is None:
        blobs = img.find_blobs(threshold,
                               invert=if_invert,
                               x_stride=x_stride,
                               y_stride=y_stride,
                               area_threshold=area_threshold,
                               pixels_threshold=pixels_threshold,
                               merge=merge)
    else:
        blobs = img.find_blobs(threshold,
                               roi=roi,
                               invert=if_invert,
                               x_stride=x_stride,
                               y_stride=y_stride,
                               area_threshold=area_threshold,
                               pixels_threshold=pixels_threshold,
                               merge=merge)
    max_blob = find_max_blob(blobs)
    if max_blob:
        # if (CenterList[0] < max_blob.cx() < (CenterList[0] + CenterList[2]) and
        # CenterList[1] < max_blob.cy() < (CenterList[1] + CenterList[3]) and
        # (max_blob.x() > RegionLimit[0] and max_blob.x() + max_blob.w() < RegionLimit[2]) and
        #(max_blob.y() > RegionLimit[1] and max_blob.y() + max_blob.h() < RegionLimit[3])

        # ):
        if (CenterList[0] < max_blob.cx() < (CenterList[0] + CenterList[2]) and
                CenterList[1] < max_blob.cy() < (CenterList[1] + CenterList[3])
            ):
            if DebugSwitchDict['ShowInfo']:
                img.draw_rectangle(max_blob.rect())
                img.draw_cross(max_blob.cx(), max_blob.cy())
            if max_blob.roundness() > 0.65 and max_blob.density() < 0.65:  # 可以近似为圆 且中心存在空洞（像素密度)
                img.draw_string(max_blob.x(), max_blob.y(),
                                f'{max_blob.roundness():.2f}\n {max_blob.density():.2f} \n RoundRing',
                                scale=2.0,
                                color=(255, 0, 0)
                                )

                TargetDict['Result'] = 0x02
                print("得到圆环")
            else:  # 更近似为矩形
                TargetDict['Result'] = 0x01
                img.draw_string(max_blob.x(), max_blob.y(),
                                f'{max_blob.roundness():.2f}\n {max_blob.density():.2f} \n Rectangle',
                                scale=2.0,
                                color=(0, 255, 0)
                                )
                print("得到矩形")
        else:
            TargetDict['Result'] = 0X66
    else:
        TargetDict['Result'] = 0X66
    Uart_Reply()

# 调试函数


def SetTarget(Mode=2, TargetColor=0, Switch=True):
    if not Switch:
        TargetDict['Switch'] = False
        return
    else:
        TargetDict['Switch'] = True
        TargetDict['Mode'] = Mode
        TargetDict['Target'] = TargetColor


def DataTransmit(if_Lock=False, SendTimes=4):
    if not if_Lock:  # 没有上锁
        UartDict['Lock'] = if_Lock  # 解锁
        UartDict['SendTimes'] = SendTimes  # 设置回传次数
    else:
        UartDict['Lock'] = True  # 关闭回传


def Uart_TransmitTest(ListObj=None):
    if ListObj is not None:
        dataLen = len(ListObj)
        cmd = struct.pack('<' + 'B' * (dataLen + 1),
                          CommunicationDict['QR_Head'],
                          *ListObj
                          )
        print(cmd)


# 色域传递修改
def EditThreshold(EditId=2, NewThreshold=[], replace2id=-1):
    """
    :param EditId: 圆盘机(2)  还是 阶梯(6)
    :param NewThreshold: 新的色域列表【单个/全部】
    :param replace2id: 取代哪一个 -1代表三个都写入
    """
    global ThresholdList, StairThresholdList
    if replace2id == -1:
        if len(NewThreshold) == 3:  # 三个色域
            if EditId == 2:
                ThresholdList = NewThreshold
            elif EditId == 6:
                StairThresholdList = NewThreshold
    elif replace2id in [0, 1, 2]:
        if len(NewThreshold) == 6:  # 单色域中有6个值
            if EditId == 2:
                ThresholdList[replace2id] = NewThreshold
            elif EditId == 6:
                StairThresholdList[replace2id] = NewThreshold


# 运行函数
def Work_Func(img2handle, if_draw=True):
    global StairThresholdList
    if True:
        cmd = struct.pack('<BB',
                          0x01,
                          0x02)
        uart.write(cmd)
        print('数据输出')
        time.sleep(100)
        return
    if TargetDict['Switch']:
        if TargetDict['Mode'] == 2:  # 圆盘机
            if TargetDict['Target'] in [0, 1, 2]:
                Find_TargetBlob(
                    img2handle, ThresholdList[TargetDict['Target']])
        elif TargetDict['Mode'] == 6:
            if TargetDict['Target'] == 0:  # 红色需要反转阈值
                StairFIndTarget(
                    img2handle, StairThresholdList[TargetDict['Target']], if_invert=True)
            elif TargetDict['Target'] == 1:
                StairFIndTarget(
                    img2handle, StairThresholdList[TargetDict['Target']], if_invert=True)
    if if_draw:
        DrawRegion(img2handle)


if __name__ == "__main__":
    print('模块调试')
    # GloablInit()
    # while True:
    # uart_receive()
    # clock.tick()
    # img = sensor.snapshot()
    # DrawRegion(img)
    # print(clock.fps())
