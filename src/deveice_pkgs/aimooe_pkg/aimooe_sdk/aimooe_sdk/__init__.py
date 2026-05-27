#!/usr/bin/env python3.10

""" import ctypes

# Load the shared library (.so file)
aim = ctypes.CDLL('AimPosition.so')

from enum import Enum

class E_Interface(Enum):
    I_USB      = 0 # USB通讯接口
    I_ETHERNET = 1 # 以太网通讯接口
    I_WIFI     = 2 # WIFI通讯接口

class E_SystemCommand(Enum):
    SC_COLLISION_DISABLE    = 0 # 关闭碰撞检测（默认打开），已有的碰撞发生警告会自动清除
    SC_COLLISION_ENABLE     = 1 # 打开碰撞检测（默认打开），检测到碰撞时，系统状态信息中能读取到碰撞发生的警告，
                                # 此时碰撞检测暂停，只有碰撞发生的警告被清除后才能继续进碰撞检测。
								# 另外，可通过Aim_SetCollisinoDetectLevel()函数设置碰撞检测的灵敏度。
    SC_COLLISION_INFO_CLEAR = 2 # 清除碰撞发生的警告
    SC_IRLED_ON             = 3 # 打开红外照明（默认打开）, 跟踪定位标记点时需要打开红外照明
    SC_IRLED_OFF            = 4 # 关闭红外照明（默认打开），在不需要红外照明时，关掉照明环可减少功耗，延长寿命
    SC_LASER_ON             = 5 # 打开定位激光（默认关闭）
    SC_LASER_OFF            = 6 # 关闭定位激光（默认关闭）
    SC_LCD_PAGE_SUBPIXEL    = 7
    SC_LCD_PAGE_COLOR       = 8
    SC_LCD_ON               = 9
    SC_LCD_OFF              = 10
    SC_AF_CONTINUOUSLY_ON   = 11
    SC_AF_CONTINUOUSLY_OFF  = 12
    SC_AF_SINGLE            = 13
    SC_AF_FIX_INFINITY      = 14
    SC_AF_EXP_AUTO_ON       = 15
    SC_AF_EXP_AUTO_OFF      = 16
    SC_AF_RESTART           = 17
    SC_DUALCAM_AUTO_EXP_ON  = 18
    SC_DUALCAM_AUTO_EXP_OFF = 19
    SC_DUALCAM_POWER_ON     = 20
    SC_DUALCAM_POWER_OFF    = 21
    SC_DUALCAM_SYNC_TRIG    = 22
    SC_NONE                 = 23

class E_DataType(Enum):
    DT_NONE                  = 0 # 不读取数据（开机默认）		
    DT_INFO                  = 1 # 仅读取信息数据（包括系统状态和三维坐标）
    DT_MARKER_INFO_WITH_WIFI = 2
    DT_STATUS_INFO           = 3
    DT_IMGDUAL               = 4
    DT_IMGCOLOR              = 5
    DT_INFO_IMGDUAL          = 6
    DT_INFO_IMGCOLOR         = 7
    DT_INFO_IMGDUAL_IMGCOLOR = 8
	
class E_ReturnValue(Enum):
    AIMOOE_ERROR          = -1
    AIMOOE_OK             = 0
    AIMOOE_CONNECT_ERROR  = 1
    AIMOOE_NOT_CONNECT    = 2
    AIMOOE_READ_FAULT     = 3
    AIMOOE_WRITE_FAULT    = 4
    AIMOOE_NOT_REFLASH    = 5
    AIMOOE_INITIAL_FAIL   = 6
    AIMOOE_HANDLE_IS_NULL = 7

# Define the function interface

# Aim_API_Initial - aimHandle
Aim_API_Initial = aim.Aim_API_Initial
Aim_API_Initial.restype = ctypes.py_object


# GetLibVersion
Aim_API_GetLibVersion = aim.Aim_API_GetLibVersion
Aim_API_GetLibVersion.restype = ctypes.c_char_p

# pospara
T_AIMPOS_DATAPARA = aim.T_AIMPOS_DATAPARA

# markerinfo
T_MarkerInfo = aim.T_MarkerInfo

# hardware
T_AimPosStatusInfo = aim.T_AimPosStatusInfo

# resDataSingle
T_AimToolDataResultSingle = aim.T_AimToolDataResultSingle

# manufactureInfo
T_ManufactureInfo = aim.T_ManufactureInfo

# mtool
T_AimToolDataResult = aim.T_AimToolDataResult

# Aim_ConnectDevice
Aim_ConnectDevice = aim.Aim_ConnectDevice
Aim_ConnectDevice.argtypes = [Aim_API_Initial.restype, E_Interface.I_USB.restype, T_AIMPOS_DATAPARA.restype]
Aim_ConnectDevice.restype = E_ReturnValue.AIMOOE_OK.restype

# SetAcquireData
Aim_SetAcquireData = aim.Aim_SetAcquireData
Aim_SetAcquireData.argtypes = [ctypes.c_void_p,ctypes.py_object,ctypes.py_object]

# GetMarkerAndStatusFromHardware
Aim_GetMarkerAndStatusFromHardware = aim.Aim_GetMarkerAndStatusFromHardware
Aim_GetMarkerAndStatusFromHardware.argtypes = [Aim_API_Initial.restype, E_Interface.I_USB.restype, T_MarkerInfo.restype, T_AimPosStatusInfo.restype]

# SetToolInfoFilePath
Aim_SetToolInfoFilePath = aim.Aim_SetToolInfoFilePath
Aim_SetToolInfoFilePath.argtypes = [Aim_API_Initial.restype, ctypes.c_char_p]

# SetSystemCommand
Aim_SetSystemCommand = aim.Aim_SetSystemCommand
Aim_SetSystemCommand.argtypes = [Aim_API_Initial.restype, E_SystemCommand.SC_COLLISION_DISABLE.restype] 
"""