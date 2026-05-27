# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import sys
import time

import os
package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

from aimooe_sdk import AimPosition as ap

import numpy as np 
from scipy.spatial.transform import Rotation

class progress_bar:

    def __init__(self,msg,bar_length) -> None:
        self.msg = msg
        self.bar_length = bar_length

    def bar(self,progress):
        block = int(round(self.bar_length * progress))
        status = f"{self.msg}:[{'#' * block}{'-' * (self.bar_length - block)}] {round(progress * 100, 2)}%"
        sys.stdout.write("\r" + status)
        sys.stdout.flush()

class tracker:

    def __init__(self,tool_path) -> None:
        self.tool_path = tool_path
        print(ap.Aim_API_GetLibVersion())
        self.aimHandle = None
        self.aimHandle = ap.Aim_API_Initial()
        assert self.aimHandle is not None
        self.interfaceType = ap.E_Interface.I_USB
        pospara = ap.T_AIMPOS_DATAPARA()

        # connect
        try:
            result = ap.Aim_ConnectDevice(self.aimHandle, self.interfaceType, pospara)
        except Exception as e:
            print(f"Cannot connect to Aimooe device: {e}")

        assert result == ap.E_ReturnValue.AIMOOE_OK
        self.markerinfo = ap.T_MarkerInfo()
        self.hardware = ap.T_AimPosStatusInfo()

        # check tool
        ap.Aim_SetToolInfoFilePath(self.aimHandle,self.tool_path)
        print("工具路径：",ap.Aim_GetToolInfoFilePath(self.aimHandle))
        
        toolSize = ap.Aim_GetCountOfToolInfo(self.aimHandle)
        print("当前目录工具数量：",toolSize)

        if toolSize != 0:
            res = ap.Aim_GetAllToolFilesBaseInfo(self.aimHandle, toolSize)
            if len(res)!=0:
                for i in range(toolSize):
                    print("工具名字：",res[i]['name'])
            else:
                print("当前目录没有工具，请检查工具路径")

        # print to screen right now
        sys.stdout.flush()

    def __del__(self) -> None:
        ap.Aim_API_Close(self.aimHandle)
        print("Aimooe closed.")

    def read_markers(self):
        """
        读取散点数据
        """
        while True:
            r=ap.Aim_GetMarkerAndStatusFromHardware(self.aimHandle,self.interfaceType,self.markerinfo,self.hardware)
            if r==ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
                time.sleep(0.01)
            else:
                break
        if r==ap.E_ReturnValue.AIMOOE_OK:
            markerNum = self.markerinfo.MarkerNumber
            print(f"开机后第{self.markerinfo.ID}次采集")
            print(f"标记点背景光(1:正常，0：不正常)：{self.markerinfo.MarkerBGLightStatus == ap.E_BackgroundLightStatus.BG_LIGHT_OK}")
            print(f"检测到标记点数量：{markerNum}")
            for i in range(markerNum):
                print(f"{i}:({self.markerinfo.MarkerCoordinate[i * 3 + 0]}, {self.markerinfo.MarkerCoordinate[i * 3 + 1]}, {self.markerinfo.MarkerCoordinate[i * 3 + 2]})")

            if self.markerinfo.PhantomMarkerGroupNumber > 0:  # 存在幻影点
                # 根据幻影点的总分组数初始化每组幻影点的存放空间
                PantomMarkerID = [[] for _ in range(self.markerinfo.PhantomMarkerGroupNumber)]
        
                # 遍历幻影点警示数组，把幻影点归类到对应分组中
                for j in range(markerNum):
                    WarningValue = self.markerinfo.PhantomMarkerWarning[j]
                    if WarningValue > 0:  # 表示第j个点为幻影点
                        PantomMarkerID[WarningValue - 1].append(j)  # 把该点归类到对应的幻影点分组
        
                # 输出
                print(f"可能存在的幻影点共有{self.markerinfo.PhantomMarkerGroupNumber}组。")
                for j in range(self.markerinfo.PhantomMarkerGroupNumber):
                    print(f"属于第{j + 1}组的点序号：", end="")
                    for i in PantomMarkerID[j]:
                        print(i, end="  ")
                    print()
            else:
                print("本次数据中不含幻影点")
        elif r==ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
            print("请确认所要获取的数据已通过Aim_SetAcquireData函数正确设置并重试!")

    def read_markers_info(self):
        """
        读取散点数据，并将每个标记点的参数存储在一个列表中返回
        返回格式: List[Dict]，每个字典包含一个标记点的详细信息
        """
        markers_list = []  # 初始化一个空列表用于存储所有标记点信息

        while True:
            r = ap.Aim_GetMarkerAndStatusFromHardware(self.aimHandle, self.interfaceType, self.markerinfo, self.hardware)
            if r == ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
                time.sleep(0.01)
            else:
                break
                
        if r == ap.E_ReturnValue.AIMOOE_OK:
            markerNum = self.markerinfo.MarkerNumber
            print(f"开机后第{self.markerinfo.ID}次采集")
            print(f"标记点背景光(1:正常，0：不正常)：{self.markerinfo.MarkerBGLightStatus == ap.E_BackgroundLightStatus.BG_LIGHT_OK}")
            print(f"检测到标记点数量：{markerNum}")
            
            # 收集所有标记点的基本信息
            for i in range(markerNum):
                marker_info = {
                    'index': i,
                    'x': self.markerinfo.MarkerCoordinate[i * 3 + 0],
                    'y': self.markerinfo.MarkerCoordinate[i * 3 + 1],
                    'z': self.markerinfo.MarkerCoordinate[i * 3 + 2],
                    'is_phantom': False,
                    'phantom_group': None
                }
                markers_list.append(marker_info)
                print(f"{i}:({marker_info['x']}, {marker_info['y']}, {marker_info['z']})")

            # 处理幻影点信息
            if self.markerinfo.PhantomMarkerGroupNumber > 0:
                PantomMarkerID = [[] for _ in range(self.markerinfo.PhantomMarkerGroupNumber)]
                
                for j in range(markerNum):
                    WarningValue = self.markerinfo.PhantomMarkerWarning[j]
                    if WarningValue > 0:
                        PantomMarkerID[WarningValue - 1].append(j)
                        # 更新标记点的幻影信息
                        markers_list[j]['is_phantom'] = True
                        markers_list[j]['phantom_group'] = WarningValue
                
                print(f"可能存在的幻影点共有{self.markerinfo.PhantomMarkerGroupNumber}组。")
                for j in range(self.markerinfo.PhantomMarkerGroupNumber):
                    print(f"属于第{j + 1}组的点序号：", end="")
                    for i in PantomMarkerID[j]:
                        print(i, end="  ")
                    print()
            else:
                print("本次数据中不含幻影点")
                
        elif r == ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
            print("请确认所要获取的数据已通过Aim_SetAcquireData函数正确设置并重试!")
        
        return markers_list  # 返回包含所有标记点信息的列表

    def require_tools(self) -> list:
        """
        将获得的每个工具的参数存储在list里面
        每个工具的参数依次包含:toolname:string pose_axang:np.array(1,6) MeanError:float
        """
        # read data
        r = ap.Aim_GetMarkerAndStatusFromHardware(self.aimHandle,self.interfaceType,self.markerinfo,self.hardware)
        if r == ap.E_ReturnValue.AIMOOE_NOT_REFLASH or (r != ap.E_ReturnValue.AIMOOE_OK):
            return    
            
        mtoolsrlt = ap.T_AimToolDataResult()

        r = ap.Aim_FindToolInfo(self.aimHandle,self.markerinfo,mtoolsrlt,0)
        if r != ap.E_ReturnValue.AIMOOE_OK:
            return
        
        # save recorded data
        data_list = []
        
        prlt = mtoolsrlt
        while (prlt!=None) and prlt.validflag:
            p_origin = np.array([[prlt.OriginCoor[0]],
                                 [prlt.OriginCoor[1]],
                                 [prlt.OriginCoor[2]]])
            rotm_origin = Rotation.from_rotvec(np.array([prlt.rotationvector[0],
                                                         prlt.rotationvector[1],
                                                         prlt.rotationvector[2]]))
            p_tool_origin = np.array([[prlt.toolCstip[0]],
                                      [prlt.toolCstip[1]],
                                      [prlt.toolCstip[2]]])
            rotm_tool_origin = Rotation.from_rotvec(np.array([prlt.toolCsmid[0],
                                                              prlt.toolCsmid[1],
                                                              prlt.toolCsmid[2]]))

            p_tool = np.dot(rotm_origin.as_matrix(),p_tool_origin) + p_origin
            rotm_tool = np.dot(rotm_origin.as_matrix(),rotm_tool_origin.as_matrix())
            rotm = Rotation.from_matrix(rotm_tool)
            r_tool = rotm.as_rotvec()

            msg = []
            msg.append(prlt.toolname)
            pose_axang = np.array([float(p_tool[0]),float(p_tool[1]),float(p_tool[2]),
                                   float(r_tool[0]),float(r_tool[1]),float(r_tool[2])])
            msg.append(pose_axang)
            msg.append(float(prlt.MeanError))
            data_list.append(msg)

            pnext = prlt.next
            del prlt
            prlt = pnext

        # if len(residualIndex) <= 0:
        return data_list

    def register_4Ptool(self, tool_name):
        """
        制作四点工具
        """
        mToolMadeinfo = ap.t_ToolMadeProInfo()
        r = ap.Aim_SetToolInfoFilePath(self.aimHandle,self.tool_path)

        r = ap.Aim_InitToolMadeInfo(self.aimHandle,4,tool_name)
        if r!=ap.E_ReturnValue.AIMOOE_OK:
            print("初始化失败！")
            return
        cntTimes = 0
        progress = progress_bar("进度",30)
        while True:
            time.sleep(0.01)
            cntTimes = cntTimes+1
            r = ap.Aim_GetMarkerAndStatusFromHardware(self.aimHandle,self.interfaceType,self.markerinfo,self.hardware)
            if r!=ap.E_ReturnValue.AIMOOE_OK:
                continue
            r = ap.Aim_ProceedToolMade(self.aimHandle,self.markerinfo,mToolMadeinfo)
            if cntTimes > 1:
                # print("进度",mToolMadeinfo.madeRate * 100)
                progress.bar(mToolMadeinfo.madeRate)            
            if mToolMadeinfo.isMadeProFinished == True and cntTimes >= 10:
                break
        del progress
        if mToolMadeinfo.isMadeProFinished == True:
            ap.Aim_SaveToolMadeRlt(self.aimHandle,True)
            print("保存工具文件: ",tool_name,".aimtool 成功!",sep='')
            print("工具路径：",ap.Aim_GetToolInfoFilePath(self.aimHandle))
        else:
            ap.Aim_SaveToolMadeRlt(self.aimHandle,False)
            print("保存工具文件失败!")
    
    def calibrate_4Ptool(self, tool_name):
        """
        校准指定工具
        """
        mToolFixinfo = ap.t_ToolFixProInfo()
        r = ap.Aim_SetToolInfoFilePath(self.aimHandle,self.tool_path)
        print("工具路径：",ap.Aim_GetToolInfoFilePath(self.aimHandle))
        mToolFixinfo.totalmarkcnt = ap.Aim_InitToolSelfCalibrationWithToolId(self.aimHandle,tool_name)
        if mToolFixinfo.totalmarkcnt == -1:
            print("找不到工具:",tool_name)
            return
        cntTimes = 0
        progress = progress_bar("进度",30)
        while True:
            time.sleep(0.01)
            cntTimes = cntTimes + 1
            r = ap.Aim_GetMarkerAndStatusFromHardware(self.aimHandle,self.interfaceType,self.markerinfo,self.hardware)
            if r!=ap.E_ReturnValue.AIMOOE_OK:
                continue
            r = ap.Aim_ProceedToolSelfCalibration(self.aimHandle,self.markerinfo,mToolFixinfo)
            print("有效校准次数:",mToolFixinfo.isValidFixCnt)
            progress.bar(mToolFixinfo.isValidFixCnt / float(100))  
            
            if mToolFixinfo.isCalibrateFinished == True and cntTimes >= 10:
                break
        # 打印误差
        print("误差：",mToolFixinfo.MatchError)
        del progress
        if mToolFixinfo.isCalibrateFinished and mToolFixinfo.MatchError<0.5:
            ap.Aim_SaveToolSelfCalibration(self.aimHandle,ap.E_ToolFixRlt.eToolFixSave)
            print("工具校准完成，精度（mm）:",mToolFixinfo.MatchError)
        else:
            ap.Aim_SaveToolSelfCalibration(self.aimHandle,ap.E_ToolFixRlt.eToolFixCancle)
            print("工具校准未成功")

    def locate_tip(self, tool_name):
        """
        针尖注册
        """
        r = ap.Aim_SetToolInfoFilePath(self.aimHandle,self.tool_path)
        
        toolTipInfo = ap.T_ToolTipPivotInfo()
        toolTipInfo.isPivotFinished = False

        r = ap.Aim_InitToolTipPivotWithToolId(self.aimHandle,tool_name,True)
        if r != ap.E_ReturnValue.AIMOOE_OK:
            return
        cntTimes = 0
        time.sleep(4)
        progress = progress_bar("进度",30)
        while True:
            time.sleep(0.1)
            cntTimes = cntTimes + 1
            r = ap.Aim_GetMarkerAndStatusFromHardware(self.aimHandle,self.interfaceType,self.markerinfo,self.hardware)
            if r!=ap.E_ReturnValue.AIMOOE_OK:
                continue
            r = ap.Aim_ProceedToolTipPivot(self.aimHandle,self.markerinfo,toolTipInfo)
            print("进度:",int(toolTipInfo.pivotRate*100))
            progress.bar(toolTipInfo.pivotRate)  
            
            if toolTipInfo.isPivotFinished == True and cntTimes >= 10:
                break
        del progress
        if toolTipInfo.isPivotFinished and toolTipInfo.pivotMeanError<1:
            ap.Aim_SaveToolTipCalibration(self.aimHandle)
            print("工具校准完成, 精度(mm):",toolTipInfo.pivotMeanError)
            print("保存工具文件: ",tool_name,".aimtool 成功!",sep='')
            print("工具路径：",ap.Aim_GetToolInfoFilePath(self.aimHandle))
            
        else:
            print("工具针尖注册不成功,误差大于1mm")

    def Aim_SetDualExpTime(self):
        """
        设置双目相机曝光值(左右均为1000)
        """
        val_low = 1000
        ap.Aim_SetDualExpTime(self.aimHandle,self.interfaceType,val_low)
        time.sleep(1.0)
        while 1:
            val = input(f"Enter number (-100 ~ 100) to adjust the min Exptime ({val_low}), 0 to finish: ")
            if int(val) == 0:
                break
            val_low += int(val)
            ap.Aim_SetDualExpTime(self.aimHandle,self.interfaceType,val_low)
            time.sleep(1.0)
        val_high = val_low
        while 1:
            val = input(f"Enter number (-100 ~ 100) to adjust the max Exptime ({val_high}), 0 to finish: ")
            if int(val) == 0:
                break
            val_high += int(val)
            ap.Aim_SetDualExpTime(self.aimHandle,self.interfaceType,val_high)
            time.sleep(1.0)
        val = int(float(val_low+val_high)/2.0)
        ap.Aim_SetDualExpTime(self.aimHandle,self.interfaceType,val)
        time.sleep(1.0)
        print('The DualExpTime is set to ', str(val), '.')

def main(args=None):
    obj = tracker(package_path+'/Aimtools/') 
    obj.Aim_SetDualExpTime()

if __name__ == '__main__':

    main()    