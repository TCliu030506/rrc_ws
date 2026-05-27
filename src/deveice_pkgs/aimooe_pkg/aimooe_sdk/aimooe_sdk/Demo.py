# -*- coding: utf-8 -*-
"""
Created on Thu Jul 13 16:59:40 2023

@author: Aimooe1
"""

import os
module_path = os.path.expanduser("~Lab_WS/zzrobot_ws/src/aimooe_pkg/aimooe_sdk")

from aimooe_sdk import AimPosition as ap

import time
from PIL import Image
import numpy as np

def caseTitle():
    print("请输入指令号码（01-99）并按回车键：")
    # print("00.重新连接设备")
    print("01.打开激光灯")
    print("02.关闭激光灯")
    print("03.打开红外照明环")
    print("04.关闭红外照明环")
    print("05.打开碰撞检测")
    print("06.关闭碰撞检测")
    print("07.清除碰撞标志")
    print("08.设置碰撞检测灵敏度为1（最灵敏）")
    print("09.设置碰撞检测灵敏度为10（最不灵敏）")
    
    print("10.设置从定位仪获取的数据为空")
    print("11.设置从定位仪获取的数据为信息数据（含系统状态和三维坐标）")
    # if EI != ap.E_ReturnValue.I_WIFI:
    #     print("12.设置从定位仪获取的数据为双目灰度图像数据")
    #     print("13.设置从定位仪获取的数据为中间彩色图像数据")
    #     print("14.设置从定位仪获取的数据为信息数据和双目灰度图像数据")
    #     print("15.设置从定位仪获取的数据为信息数据和中间彩色图像数据")
    #     print("16.设置从定位仪获取的数据为信息数据，双目灰度图像数据和中间彩色图像数据")
    # else:
    #     print("17.设置从定位仪获取的数据为三维坐标数据")
    #     print("18.设置从定位仪获取的数据为系统状态数据")
    
    print("20.关闭触控显示屏")
    print("21.打开触控显示屏")
    print("22.复位触控显示屏")
    print("23.切换触控屏页面到：标记点检测页面")
    print("25.切换触控屏页面到：中间彩色图像页面")
    print("26.设置触控屏标记点检测页面显示的点为未经处理的点")
    print("27.设置触控屏标记点检测页面显示的点为处理后的点")
    
    print("30.启动中间相机自动曝光")
    print("31.关闭中间相机自动曝光")
    print("32.设置中间相机曝光值为4000")
    print("33.启动中间相机持续对焦")
    print("34.关闭中间相机持续对焦")
    print("35.启动中间相机单次对焦")
    print("36.中间相机固定焦距到无穷远")
    
    print("40.启动双目自动曝光(开机默认关闭)")
    print("41.关闭双目自动曝光(开机默认关闭)")
    print("42.设置双目相机曝光值（左右均为1000）")
    print("43.以采集到的标记点的Z轴值为距离设置曝光值（多个点时取中值）")
    print("44.设置闪光灯延时")
    
    print("50.获取系统状态信息")
    print("51.获取标记点三维坐标信息")
    # if EI != ap.E_ReturnValue.I_WIFI:
    #     print("52.获取左右相机图像Left.bmp和Right.bmp到CameraImage文件夹")
    #     print("53.获取中间彩色相机图像Color.bmp到CameraImage文件夹")
    print("54.获取当前使用的IP地址信息")
    print("55.获取出厂信息")
    print("56.获取设备Mac地址")
    
    print("60.获取某路径下的定位工具文件，并获取工具的信息")
    print("61.获取某特定的工具的具体信息")
    print("62.空间配准")
    print("63.制作4点工具")
    print("64.校准工具")
    print("65.工具针尖注册（注册板）")
    print("75.工具针尖注册（绕点旋转）")
    print("76.工具注册--坐标转换版本（把工具文件的点坐标转到注册版工具文件的坐标系上。）")
    
    print("66.AAK精度工具测试")
    print("67.检查工具文件")
    
    print("98.重新选择连接方式")
    #print("99.退出")

def whichcase(value):
    
    if value == 1:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_LASER_ON)
    elif value == 2:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_LASER_OFF)
    elif value == 3:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_IRLED_ON)
    elif value == 4:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_IRLED_OFF)
    elif value == 5:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_COLLISION_ENABLE)
    elif value == 6:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_COLLISION_DISABLE)
    elif value == 7:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_COLLISION_INFO_CLEAR)
    elif value == 8:
        r=ap.Aim_SetCollisinoDetectLevel(aimHandle,interfaceType,1)
    elif value == 9:
        r=ap.Aim_SetCollisinoDetectLevel(aimHandle,interfaceType,10)
    elif value == 10:
        r=ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_NONE)
    elif value == 11:
        r=ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_INFO)
    elif value == 12:
        r=ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_IMGDUAL)
    elif value == 13:
        r=ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_IMGCOLOR)
    elif value == 14:
        r=ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_INFO_IMGDUAL)
    elif value == 15:
        r=ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_INFO_IMGCOLOR)
    elif value == 16:
        r=ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_INFO_IMGDUAL_IMGCOLOR)
    elif value == 17:
        r=ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_MARKER_INFO_WITH_WIFI)
    elif value == 18:
        r=ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_STATUS_INFO)
    
    elif value == 20:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_LCD_OFF)
    elif value == 21:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_LCD_ON)
        
    elif value == 23:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_LCD_PAGE_SUBPIXEL)
        
    elif value == 25:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_LCD_PAGE_COLOR)
    elif value == 26:
        r=ap.Aim_SetLCDShowRawPoint(aimHandle,interfaceType,True)
    elif value == 27:
        r=ap.Aim_SetLCDShowRawPoint(aimHandle,interfaceType,False)
        
    elif value == 30:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_AF_EXP_AUTO_ON)
    elif value == 31:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_AF_EXP_AUTO_OFF)    
    elif value == 32:
        r=ap.Aim_SetColorExpTime(aimHandle,interfaceType,4000)
    elif value == 33:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_AF_CONTINUOUSLY_ON)    
    elif value == 34:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_AF_CONTINUOUSLY_OFF)    
    elif value == 35:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_AF_SINGLE)    
    elif value == 36:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_AF_FIX_INFINITY)    
        
    elif value == 40:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_DUALCAM_AUTO_EXP_ON)  
    elif value == 41:
        r=ap.Aim_SetSystemCommand(aimHandle,interfaceType,ap.E_SystemCommand.SC_DUALCAM_AUTO_EXP_ON)
    elif value == 42:
        r=ap.Aim_SetDualExpTime(aimHandle,interfaceType,1000)
        
    elif value == 43:
        r=ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_NONE)
        if r==ap.E_ReturnValue.AIMOOE_OK:
            r=ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            if r==ap.E_ReturnValue.AIMOOE_OK:
                ZDis = []
                for i in range(markerinfo.MarkerNumber):
                    ZDis.append(markerinfo.MarkerCoordinate[i * 3 + 2])
                
                ZDis.sort(reverse=True)
                mid = len(ZDis) // 2
                dis = ZDis[mid]
                r=ap.Aim_SetDualExpTimeByDistance(aimHandle, interfaceType, dis)
    elif value == 44:
        r=ap.Aim_SetFlashDelay(aimHandle, interfaceType, 1, 0)
        
    elif value == 50:
        r=ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
        if r==ap.E_ReturnValue.AIMOOE_OK:
            print("当前碰撞状态（0：未碰撞，1：检测到碰撞，2，未开启检测）：", hardware.CollisionStatus)
            print("CPU温度(℃)：", hardware.Tcpu, "  ", "主板温度(℃)：", hardware.Tpcb)
            print("硬件状态是否正常(1:正常，0：不正常)：", hardware.HardwareStatus == ap.E_HardwareStatus.HW_OK)
            print("帧率   左相机：", int(hardware.LeftCamFps), "   右相机：", int(hardware.RightCamFps),
                  "   彩色相机：", int(hardware.ColorCamFps), "   触控屏：", int(hardware.LCDFps))
            print("曝光时间   左相机：", hardware.ExposureTimeLeftCam, "   右相机：", hardware.ExposureTimeRightCam)
        elif r==ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
            print("请确认所要获取的数据已通过Aim_SetAcquireData函数正确设置并重试")


    elif value == 51:
        while True:
            r=ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            if r==ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
                time.sleep(0.01)
            else:
                break
        if r==ap.E_ReturnValue.AIMOOE_OK:
            markerNum = markerinfo.MarkerNumber
            print(f"开机后第{markerinfo.ID}次采集")
            print(f"标记点背景光(1:正常，0：不正常)：{markerinfo.MarkerBGLightStatus == ap.E_BackgroundLightStatus.BG_LIGHT_OK}")
            print(f"检测到标记点数量：{markerNum}")
            for i in range(markerNum):
                print(f"{i}:({markerinfo.MarkerCoordinate[i * 3 + 0]}, {markerinfo.MarkerCoordinate[i * 3 + 1]}, {markerinfo.MarkerCoordinate[i * 3 + 2]})")
    
            if markerinfo.PhantomMarkerGroupNumber > 0:  # 存在幻影点
                # 根据幻影点的总分组数初始化每组幻影点的存放空间
                PantomMarkerID = [[] for _ in range(markerinfo.PhantomMarkerGroupNumber)]
        
                # 遍历幻影点警示数组，把幻影点归类到对应分组中
                for j in range(markerNum):
                    WarningValue = markerinfo.PhantomMarkerWarning[j]
                    if WarningValue > 0:  # 表示第j个点为幻影点
                        PantomMarkerID[WarningValue - 1].append(j)  # 把该点归类到对应的幻影点分组
        
                # 输出
                print(f"可能存在的幻影点共有{markerinfo.PhantomMarkerGroupNumber}组。")
                for j in range(markerinfo.PhantomMarkerGroupNumber):
                    print(f"属于第{j + 1}组的点序号：", end="")
                    for i in PantomMarkerID[j]:
                        print(i, end="  ")
                    print()
            else:
                print("本次数据中不含幻影点")
        elif r==ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
            print("请确认所要获取的数据已通过Aim_SetAcquireData函数正确设置并重试!")
    elif value == 52:
        imgDual = ap.Aim_GetMarkerStatusAndGreyImageFromHardware(aimHandle,interfaceType,pospara,markerinfo,hardware)
        if len(imgDual)!=0:
            imgl = np.squeeze(imgDual["imageL"])
            imgr = np.squeeze(imgDual["imageR"])
            #保存图像
            leftone = Image.fromarray(imgl)
            rightone = Image.fromarray(imgr)
            
        else:
            print("保存失败,请确认所要获取的数据已通过Aim_SetAcquireData函数正确设置并重试!")
    elif value == 53:
        imgColor = ap.Aim_GetColorImageFromHardware(aimHandle,interfaceType,pospara)
        if len(imgColor)!=0:
            imgc = np.squeeze(imgColor)
            #保存图像
            colorone = Image.fromarray(imgc)      
        else:
            print("保存失败,请确认所要获取的数据已通过Aim_SetAcquireData函数正确设置并重试!")
    
    elif value == 54:
        ipAddr = ap.Aim_GetAimPositionIP(aimHandle,interfaceType)
        if len(ipAddr)!=0:
            print("当前IP:",ipAddr)
        else:
            print("请先连接定位仪!")
            
    elif value == 55:
        r=ap.Aim_GetManufactureInfo(aimHandle,interfaceType,manufactureInfo)
        ver_str = ''.join(manufactureInfo.Version)
        print("版本号",ver_str)
        print("出厂日期",manufactureInfo.Year,"/",manufactureInfo.Month,"/",manufactureInfo.Day)
        
    elif value == 56:
        ipMac = ap.Aim_GetAimPositonMacAddress(aimHandle)
        if len(ipMac)!=0:
            ipMac_ascii = [chr(i) for i in ipMac]
            ipMac_str = ''.join(ipMac_ascii)
            print("Mac地址:",ipMac_str)
        else:
            print("未知错误!")
    
    elif value == 60:
        r = ap.Aim_SetToolInfoFilePath(aimHandle,toolPath)
        print("工具路径：",ap.Aim_GetToolInfoFilePath(aimHandle))
        
        toolSize = ap.Aim_GetCountOfToolInfo(aimHandle)
        print("工具数量：",toolSize)
        
        if toolSize != 0:
            res = ap.Aim_GetAllToolFilesBaseInfo(aimHandle, toolSize)
            if len(res)!=0:
                for i in range(toolSize):
                    print("工具名字：",res[i]['name'])
            else:
                print("当前目录没有工具，请检查工具路径")
        while True:
            r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            while r == ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
                time.sleep(0.01)
                r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle, interfaceType, markerinfo,hardware)         
                if r != ap.E_ReturnValue.AIMOOE_OK:
                    break
            residualIndex = list(range(markerinfo.MarkerNumber))
            mtoolsrlt = ap.T_AimToolDataResult()
            r = ap.Aim_FindToolInfo(aimHandle,markerinfo,mtoolsrlt,0)
            if r == ap.E_ReturnValue.AIMOOE_OK:
                prlt = mtoolsrlt
                while prlt!=None:
                    if prlt.validflag:
                        PI = 3.1415962
                        print(f"找到工具: {prlt.toolname}  平均误差{prlt.MeanError} RMS误差{prlt.Rms}")
                        print(f"工具原点: {prlt.OriginCoor[0]}, {prlt.OriginCoor[1]}, {prlt.OriginCoor[2]}")
                        print(f"工具角度: {prlt.rotationvector[0] * 180 / PI}, {prlt.rotationvector[1] * 180 / PI}, {prlt.rotationvector[2] * 180 / PI}")
                        print("标记点坐标:")
                        for idx in prlt.toolptidx:
                            if idx in residualIndex:
                                residualIndex.remove(idx)
                            if idx < 0:
                                print("0 0 0")
                            else:
                                print(f"{markerinfo.MarkerCoordinate[idx * 3 + 0]} {markerinfo.MarkerCoordinate[idx * 3 + 1]} {markerinfo.MarkerCoordinate[idx * 3 + 2]}")
                        pnext = prlt.next
                        del prlt
                        prlt = pnext

                if len(residualIndex) > 0:
                    print(f"共{len(residualIndex)}个离散点：")
                    for i, j in enumerate(residualIndex):
                        print(f"{i}: {markerinfo.MarkerCoordinate[j * 3 + 0]}, {markerinfo.MarkerCoordinate[j * 3 + 1]}, {markerinfo.MarkerCoordinate[j * 3 + 2]}")
                else:
                    break
            else:
                del prlt
                break
                
        print("查找结束")
    
    elif value == 61:
        r = ap.Aim_SetToolInfoFilePath(aimHandle,toolPath)
        print("工具路径：",ap.Aim_GetToolInfoFilePath(aimHandle))
        toollist = ap.StringVector(['caliBoard', 'tip2', 'cali2','newtool'])
        while True:
            r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            while r == ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
                time.sleep(0.01)
                r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle, interfaceType, markerinfo,hardware)         
                if r != ap.E_ReturnValue.AIMOOE_OK:
                    break
            residualIndex = list(range(markerinfo.MarkerNumber))
            mtoolsrlt = ap.T_AimToolDataResult()
            r=ap.Aim_FindSpecificToolInfo(aimHandle,markerinfo,toollist,mtoolsrlt,3)
            if r == ap.E_ReturnValue.AIMOOE_OK:
                prlt = mtoolsrlt
                while prlt!=None:
                    if prlt.validflag:
                        PI = 3.1415962
                        print(f"找到工具: {prlt.toolname}  平均误差{prlt.MeanError} RMS误差{prlt.Rms}")
                        print(f"工具原点: {prlt.OriginCoor[0]}, {prlt.OriginCoor[1]}, {prlt.OriginCoor[2]}")
                        print(f"工具角度: {prlt.rotationvector[0] * 180 / PI}, {prlt.rotationvector[1] * 180 / PI}, {prlt.rotationvector[2] * 180 / PI}")
                        print("标记点坐标:")
                        for idx in prlt.toolptidx:
                            if idx in residualIndex:
                                residualIndex.remove(idx)
                            if idx < 0:
                                print("0 0 0")
                            else:
                                print(f"{markerinfo.MarkerCoordinate[idx * 3 + 0]} {markerinfo.MarkerCoordinate[idx * 3 + 1]} {markerinfo.MarkerCoordinate[idx * 3 + 2]}")
                        pnext = prlt.next
                        del prlt
                        prlt = pnext
                if len(residualIndex) > 0:
                    print(f"共{len(residualIndex)}个离散点：")
                    for i, j in enumerate(residualIndex):
                        print(f"{i}: {markerinfo.MarkerCoordinate[j * 3 + 0]}, {markerinfo.MarkerCoordinate[j * 3 + 1]}, {markerinfo.MarkerCoordinate[j * 3 + 2]}")
                break
            else:
                del prlt
                break
            
        print("查找结束")  
    
    elif value == 62:
        CTMarkerPoint = np.array([
                                    [-189.789388859475, -83.4998068907247, 2247.92167116227],
                                    [-130.869739724100, -127.612744935898, 2229.51491037209],
                                    [-170.628261385302, -172.885513685218, 2196.26853365091],
                                    [-230.927341230568, -116.157778943286, 2222.15472458089]
                                ], dtype=np.float32)
        CTDstPoint = np.array([-230.927341230568, -116.157778943286, 2222.15472458089], dtype=np.float32)
        
        r=ap.Aim_InitMappingPointSetsForMarkerSpaceReg(aimHandle,CTMarkerPoint,4)
        if r==ap.E_ReturnValue.AIMOOE_OK:
            while True:
                r=ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
                while r == ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
                    time.sleep(0.01)
                    r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle, interfaceType, markerinfo,hardware)      
                    if r != ap.E_ReturnValue.AIMOOE_OK:
                        break
                prlt = ap.T_AimToolDataResult()
                r=ap.Aim_MappingPointSetsForMarkerSpaceReg(aimHandle,markerinfo,prlt,3)
                if r != ap.E_ReturnValue.AIMOOE_OK:
                    print("配准失败！")
                    continue
                dstPointInOpticalSys = [0.0, 0.0, 0.0]
                if prlt.validflag:
                    PI = 3.1415926
                    print(f"配准成功！  平均配准误差{prlt.MeanError}")
                    print("相机空间的标记点在光学定位系统中的坐标：")
                    for i, idx in enumerate(prlt.toolptidx):
                        if idx < 0:
                            print("0 0 0")
                        else:
                            print(f"{markerinfo.MarkerCoordinate[idx * 3 + 0]}, {markerinfo.MarkerCoordinate[idx * 3 + 1]}, {markerinfo.MarkerCoordinate[idx * 3 + 2]}")
                    
                    RxCTDstPoint = [0.0, 0.0, 0.0]
                    for i in range(3):
                        for j in range(3):
                            RxCTDstPoint[i] += prlt.Rto[i][j] * CTDstPoint[j]
            
                    for i in range(3):
                        dstPointInOpticalSys[i] = RxCTDstPoint[i] + prlt.Tto[i]
            
                    print(f"相机空间的目标点在光学定位系统中的坐标：({dstPointInOpticalSys[0]}, {dstPointInOpticalSys[1]}, {dstPointInOpticalSys[2]})")
                    break
                else:
                    print("配准失败！")
                    
    elif value == 63:
        mToolMadeinfo = ap.t_ToolMadeProInfo()
        r = ap.Aim_SetToolInfoFilePath(aimHandle,toolPath)
        print("工具路径：",ap.Aim_GetToolInfoFilePath(aimHandle))
        
        r = ap.Aim_InitToolMadeInfo(aimHandle,4,"AAK-right")
        if r!=ap.E_ReturnValue.AIMOOE_OK:
            print("初始化失败！")
            return
        cntTimes = 0
        while True:
            time.sleep(0.01)
            cntTimes = cntTimes+1
            r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            if r!=ap.E_ReturnValue.AIMOOE_OK:
                continue
            r = ap.Aim_ProceedToolMade(aimHandle,markerinfo,mToolMadeinfo)
            print("进度",mToolMadeinfo.madeRate * 100)
            
            if mToolMadeinfo.isMadeProFinished == True and cntTimes >= 10:
                break
        if mToolMadeinfo.isMadeProFinished == True:
            ap.Aim_SaveToolMadeRlt(aimHandle,True)
            print("保存工具文件成功!")
        else:
            ap.Aim_SaveToolMadeRlt(aimHandle,False)
            print("保存工具文件失败!")
    
    elif value == 64:
        mToolFixinfo = ap.t_ToolFixProInfo()
        r = ap.Aim_SetToolInfoFilePath(aimHandle,toolPath)
        print("工具路径：",ap.Aim_GetToolInfoFilePath(aimHandle))
        mToolFixinfo.totalmarkcnt = ap.Aim_InitToolSelfCalibrationWithToolId(aimHandle,"tip2")
        if mToolFixinfo.totalmarkcnt == -1:
            return
        cntTimes = 0
        while True:
            time.sleep(0.01)
            cntTimes = cntTimes + 1
            r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            if r!=ap.E_ReturnValue.AIMOOE_OK:
                continue
            r = ap.Aim_ProceedToolSelfCalibration(aimHandle,markerinfo,mToolFixinfo)
            print("有效校准次数:",mToolFixinfo.isValidFixCnt)
            
            if mToolFixinfo.isCalibrateFinished == True and cntTimes >= 10:
                break
        if mToolFixinfo.isCalibrateFinished and mToolFixinfo.MatchError<0.5:
            ap.Aim_SaveToolSelfCalibration(aimHandle,ap.E_ToolFixRlt.eToolFixSave)
            print("工具校准完成，精度（mm）:",mToolFixinfo.MatchError)
        else:
            ap.Aim_SaveToolSelfCalibration(aimHandle,ap.E_ToolFixRlt.eToolFixCancle)
            print("工具校准未成功")
            
    elif value == 65:
        toolTipInfo = ap.t_ToolTipCalProInfo()
        toolTipInfo.isCalibrateFinished = False
        mToolFixinfo = ap.t_ToolFixProInfo()
        r = ap.Aim_SetToolInfoFilePath(aimHandle,toolPath)
        print("工具路径：",ap.Aim_GetToolInfoFilePath(aimHandle))
        
        r = ap.Aim_InitToolTipCalibrationWithToolId(aimHandle,"CTS-B4B0-006-1","tip2")
        
        if r != ap.E_ReturnValue.AIMOOE_OK:
            return
        cntTimes = 0
        while True:
            time.sleep(0.01)
            cntTimes = cntTimes + 1
            r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            if r!=ap.E_ReturnValue.AIMOOE_OK:
                continue
            r = ap.Aim_ProceedToolTipCalibration(aimHandle,markerinfo,toolTipInfo)
            print("进度:",int(toolTipInfo.CalibrateRate*100))
            
            if toolTipInfo.isCalibrateFinished == True and cntTimes >= 10:
                break
        if toolTipInfo.isCalibrateFinished and toolTipInfo.CalRMSError<0.5:
            ap.Aim_SaveToolTipCalibration(aimHandle)
            print("工具校准完成，精度（mm）:",toolTipInfo.CalRMSError)
        else:
            print("工具针尖注册未成功")
            
    elif value == 66:
        accToolRlt = ap.T_AccuracyToolResult()
        r = ap.Aim_SetToolInfoFilePath(aimHandle,toolPath)
        print("工具路径：",ap.Aim_GetToolInfoFilePath(aimHandle))
        
        r = ap.Aim_InitAccuracyCheckTool(aimHandle,"AAK-whole","AAK-left","AAK-right")#"AAS-B8CD1","AAS-B4C1","AAS-B4D1")
        if r != ap.E_ReturnValue.AIMOOE_OK:
            return
        
        for i in range(20):
            time.sleep(0.1)
            r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            if r!=ap.E_ReturnValue.AIMOOE_OK:
                continue
            markerArr = []
            for j in range(markerinfo.MarkerNumber):
                x = markerinfo.MarkerCoordinate[j*3+0]
                y = markerinfo.MarkerCoordinate[j*3+1]
                z = markerinfo.MarkerCoordinate[j*3+2]
                markerArr.append([x,y,z])
            markerArr_np = np.array(markerArr,dtype = np.float64)
            
            r = ap.Aim_AccuracyCheckTool(aimHandle,markerArr_np,markerinfo.MarkerNumber,accToolRlt)
            if accToolRlt.validflag == True:
                print(f"{accToolRlt.Angle[0]} {accToolRlt.Angle[1]} {accToolRlt.Angle[2]} {accToolRlt.Dis}")

        accResult = ap.Aim_AccuracyCheckToolCalculateError(aimHandle)
        print(f"中心距离误差均值: {accResult[0]}  中心距离误差离散性{accResult[1]} 角度偏差均值：{accResult[2][0],accResult[2][1],accResult[2][2]}")
    
    elif value == 67:
        data = ap.T_ToolFileData()
        r = ap.Aim_CheckToolFile(aimHandle,toolPath+"tip2.aimtool",data)
        print("工具名字",data.toolname)
        print("工具类型",data.tooType)
        print("工具标记求数量",data.markerNumbers)
        
        num = data.markerNumbers
        if num > 16:
            num = 16
        
        for i in range(num):
            print(f"{i}: {data.MarkerCoordinate[i * 3 + 0]}, {data.MarkerCoordinate[i * 3 + 1]}, {data.MarkerCoordinate[i * 3 + 2]}")
        
        print("针尖:",data.tipHeadCoordinate[0],data.tipHeadCoordinate[0],data.tipHeadCoordinate[0])
        print("针体:",data.tipBodyCoordinate[1],data.tipBodyCoordinate[1],data.tipBodyCoordinate[1])
        
        if r == ap.E_ReturnValue.AIMOOE_OK:
            print("检查完成")
        else:
            print("检查失败")
            
    elif value == 75:
        r = ap.Aim_SetToolInfoFilePath(aimHandle,toolPath)
        print("工具路径：",ap.Aim_GetToolInfoFilePath(aimHandle))
        toolTipInfo = ap.T_ToolTipPivotInfo()
        toolTipInfo.isPivotFinished = False
        
        r = ap.Aim_InitToolTipPivotWithToolId(aimHandle,"tip2",True)
        if r != ap.E_ReturnValue.AIMOOE_OK:
            return
        cntTimes = 0
        time.sleep(4)
        while True:
            time.sleep(0.05)
            cntTimes = cntTimes+1
            r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            if r!=ap.E_ReturnValue.AIMOOE_OK:
                continue
            r = ap.Aim_ProceedToolTipPivot(aimHandle,markerinfo,toolTipInfo)
            print("进度:",int(toolTipInfo.pivotRate*100))
            
            if toolTipInfo.isPivotFinished == True and cntTimes >= 10:
                break
        if toolTipInfo.isPivotFinished and toolTipInfo.pivotMeanError<1:
            ap.Aim_SaveToolTipCalibration(aimHandle)
            print("工具校准完成，精度（mm）:",toolTipInfo.pivotMeanError)
            
        else:
            print("工具针尖注册不成功,误差大于1mm")
    
    elif value == 76:
        toolTipInfo = ap.t_ToolTipCalProInfo()
        toolTipInfo.isCalibrateFinished = False
        r = ap.Aim_SetToolInfoFilePath(aimHandle,toolPath)
        print("工具路径：",ap.Aim_GetToolInfoFilePath(aimHandle))
        
        r = ap.Aim_InitToolCoordinateRenewWithToolId(aimHandle,"EDGE-CALIB","PT4M-EDGE")
        if r != ap.E_ReturnValue.AIMOOE_OK:
            return
        cntTimes = 0
        while True:
            time.sleep(0.01)
            cntTimes = cntTimes+1
            r = ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            if r!=ap.E_ReturnValue.AIMOOE_OK:
                continue
            r = ap.Aim_ProceedToolCoordinateRenew(aimHandle,markerinfo,toolTipInfo)
            print("进度:",int(toolTipInfo.CalibrateRate))
            
            if toolTipInfo.isCalibrateFinished == True and cntTimes >= 1000:
                break
        if toolTipInfo.isCalibrateFinished and toolTipInfo.CalRMSError<0.5:
            ap.Aim_SaveToolCoordinateRenew(aimHandle)
            print("削骨工具注册完成，精度（mm）:",toolTipInfo.CalRMSError)
            
        else:
            print("削骨工具注册未成功")
        
# main                
if __name__ == "__main__":

    aimHandle = None
    aimHandle = ap.Aim_API_Initial()
    print(aimHandle)
    print(ap.Aim_API_GetLibVersion())
    toolPath = module_path + '/Aimtools/'
    #连接成功
    if aimHandle is not None:
        interfaceType = ap.E_Interface.I_USB
        pospara = ap.T_AIMPOS_DATAPARA()
        result = ap.Aim_ConnectDevice(aimHandle, interfaceType, pospara)
 
        if result == ap.E_ReturnValue.AIMOOE_OK:
            # 连接成功，继续其他操作
            markerinfo = ap.T_MarkerInfo()
            hardware = ap.T_AimPosStatusInfo()
            resDataSingle = ap.T_AimToolDataResultSingle()
            manufactureInfo = ap.T_ManufactureInfo()
            mtool = ap.T_AimToolDataResult()
            success = False
            ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_NONE)
            ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            ap.Aim_SetToolInfoFilePath(aimHandle,toolPath)
            # toollist = ap.StringVector(['caliBoard.aimtool', 'smallone', 'cali2'])
            # ap.Aim_FindSpecificToolInfo(aimHandle,markerinfo,toollist,mtool,3)
            while success != True:
                caseTitle()
                userCase = input("目前以USB连接定位仪成功，请输出要执行的Case:")
                whichcase(int(userCase))
            
            # ap.Aim_SetAcquireData(aimHandle,interfaceType,ap.E_DataType.DT_NONE)

            # ap.Aim_GetMarkerAndStatusFromHardware(aimHandle,interfaceType,markerinfo,hardware)
            # ap.Aim_SetToolInfoFilePath(aimHandle,'D:/AimPosAppSolutionAPI_V234-hwz/AimToolBox/Config/AimTools/')

            # toollist = ap.StringVector(['caliBoard', 'smallone', 'value3'])
            # ap.Aim_FindSingleToolInfo(aimHandle, markerinfo, toollist[0], resDataSingle,3)
            # imageC = ap.Aim_GetColorImageFromHardware(aimHandle,interfaceType,pospara)
        else:
            #连接失败
            print("连接定位失败")
    else:
        # 初始化失败
        print("初始化失败，请重启定位仪")

