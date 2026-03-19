// AimPosDemo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "windows.h"
#include "direct.h"
#include "io.h"
#include <iostream>
#include <math.h>
#include<algorithm>
#include<thread>
#include<mutex>

#include "AimPositionAPI.h"
#include "AimPositionDef.h"
//#include "vld.h"
using   namespace   std;
#ifdef _DEBUG
#ifdef _WIN64
#pragma comment(lib,"AimPosition2.3.4d.lib")
#else
#pragma comment(lib,"AimPosition2.3.4dX86.lib")
#endif
#else
#ifdef _WIN64
#pragma comment(lib,"AimPosition2.3.4.lib")
#else
#pragma comment(lib,"AimPosition2.3.4X86.lib")
#endif

#endif // _DEBUG

#define GET_DATA_FROM_HARDWARE 1

enum E_BMPBIT
{
	BMP8BIT,
	BMP16BIT,
	BMP24BIT,
};
void write_bmpheader(unsigned char *bitmap, int offset, int bytes, int value);
unsigned char *convertGrayTo24bitBmp(unsigned char *inputImg, int width, int height, int *ouputSize);
unsigned char *convertColor16To24bitBmp(unsigned char *inputImg, int width, int height, int *ouputSize);
bool saveToBmp(unsigned char *inputImg, int width, int height, char *outputFileName, E_BMPBIT bmpBit);
void testDemo();

int testFunction();
void setText();
int useUSBorEthernetorWIFI = 0;
E_Interface EI;
AimHandle aimHandle = NULL;

int main()
{

	testDemo();

	system("pause");
	return 0;
}

void testDemo()
{
	int rt = 1;
	E_ReturnValue rlt = Aim_API_Initial(aimHandle);
	if (rlt != AIMOOE_OK)
	{
		cout << "初始化失败！" << endl;
		return;
	}

	while (rt)
	{
		char buf[10];
		std::cout << "请选择连接方式（0：USB，1：以太网，2：WIFI）并按回车键：" << endl;
		std::cin.getline(buf, 3);
		useUSBorEthernetorWIFI = buf[0] - 0x30;
		switch (useUSBorEthernetorWIFI)
		{
		case 0: {EI = I_USB; }break;
		case 1: {EI = I_ETHERNET;  }break;
		case 2: {EI = I_WIFI; }break;
		default: {std::cout << "输入有误！" << endl; continue; }break;
		}
		rt = testFunction();
	}
	rlt = Aim_API_Close(aimHandle);
	if (rlt != AIMOOE_OK)
	{
		cout << "API关闭失败！" << endl;
		return;
	}
}

int testFunction()
{
	string strversion;
	char buf[10];
	int number;
	char * path = ".\\AimTools\\";
	//char * path = "C:\\AimTools\\";
	char **toolarr = NULL;


	T_AIMPOS_DATAPARA mPosDataPara;
	if (_access("./CameraImage", 0) != 0)
		_mkdir("./CameraImage");

	T_AimPosStatusInfo statusSt;
	T_MarkerInfo markerSt;
	T_ManufactureInfo manufactureInfo;
	char* ImageLeft = NULL;// new char[GRAYIMAGESIZE];
	char* ImageRight = NULL; //new char[GRAYIMAGESIZE];
	char* ImageColor = NULL; //new char[COLORIMAGESIZE];
	int graywidth = 0;
	int grayheight = 0;
	int colorwidth = 0;
	int colorheight = 0;
	thread tI;
	thread tD;
	thread tC;
	int rt;
	if (EI == I_ETHERNET)
	{
		//App-100
		//Aim_SetEthernetConnectIP(192, 168,1,10);
		//App-200
		Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
	}
	E_ReturnValue rlt = Aim_ConnectDevice(aimHandle, EI, mPosDataPara);

#if GET_DATA_FROM_HARDWARE 
	Aim_SetAcquireData(aimHandle, EI, DT_NONE);
#else
	Aim_SetAcquireData(aimHandle, EI, DT_INFO);
#endif
	if (rlt == AIMOOE_OK)
	{
		//Aim_SetMarkerParameters(aimHandle, EI, 75,1000,80);
		std::cout << "连接成功！" << endl;
		switch (mPosDataPara.devtype)
		{
		case AIMPOS_TYPE::eAP_Standard:
		{
			std::cout << "定位系统型号： 标准版" << endl;

		}	break;
		case AIMPOS_TYPE::eAP_Lite:
		{
			std::cout << "定位系统型号： 轻量版" << endl;

		}	break;
		case AIMPOS_TYPE::eAP_Ultimate:
		{
			std::cout << "定位系统型号： 旗舰版" << endl;
		}break;
		case AIMPOS_TYPE::eAP_Basic:
		{
			std::cout << "定位系统型号： 基础版" << endl;
		}break;
		case AIMPOS_TYPE::eAP_Industry:
		{
			std::cout << "定位系统型号: 工业版" << endl;
		}break;

		default:
			break;
		}

		graywidth = mPosDataPara.dualimg.width;
		grayheight = mPosDataPara.dualimg.height;
		colorwidth = mPosDataPara.colorimg.width;
		colorheight = mPosDataPara.colorimg.height;
		if (ImageLeft != NULL)
		{
			delete[]ImageLeft;
			ImageLeft = NULL;
		}
		if (ImageRight != NULL)
		{
			delete[]ImageRight;
			ImageRight = NULL;
		}
		if (ImageColor != NULL)
		{
			delete[]ImageColor;
			ImageColor = NULL;
		}
		ImageLeft = new char[graywidth*grayheight];
		ImageRight = new char[graywidth*grayheight];
		ImageColor = new char[colorwidth*colorheight * 2];
		std::cout << "按回车键继续!" << endl;
		std::cin.getline(buf, 10);

		//	Aim_SetAcquireData(aimHandle, EI, DT_NONE);




	}
	else {
		std::cout << "连接设备错误，请确认设备已连接！" << endl;
		std::cout << "按回车键继续!" << endl;
		std::cin.getline(buf, 10);
		return 1;
	}

	while (1)
	{
		setText();
		std::cin.getline(buf, 3);
		/*std::cout << buf[0] << buf[1] << endl;*/
		number = (buf[0] - 0x30) * 10 + buf[1] - 0x30;
		switch (number)
		{
		case 0:
		{
			rlt = Aim_ConnectDevice(aimHandle, EI, mPosDataPara);
#if GET_DATA_FROM_HARDWARE 
			Aim_SetAcquireData(aimHandle, EI, DT_NONE);
#else
			Aim_SetAcquireData(aimHandle, EI, DT_INFO);
#endif
			if (rlt == AIMOOE_OK)
			{

				switch (mPosDataPara.devtype)
				{
				case AIMPOS_TYPE::eAP_Standard:
				{
					std::cout << "定位系统型号： 标准版" << endl;

				}	break;
				case AIMPOS_TYPE::eAP_Lite:
				{
					std::cout << "定位系统型号： 轻量版" << endl;

				}	break;
				case AIMPOS_TYPE::eAP_Ultimate:
				{
					std::cout << "定位系统型号： 旗舰版" << endl;
				}break;
				case AIMPOS_TYPE::eAP_Basic:
				{
					std::cout << "定位系统型号： 基础版" << endl;
				}break;
				case AIMPOS_TYPE::eAP_Industry:
				{
					std::cout << "定位系统型号：AP-400 工业版" << endl;
				}break;

				default:
					break;
				}

				graywidth = mPosDataPara.dualimg.width;
				grayheight = mPosDataPara.dualimg.height;
				colorwidth = mPosDataPara.colorimg.width;
				colorheight = mPosDataPara.colorimg.height;
				if (ImageLeft != NULL)
				{
					free(ImageLeft);
					ImageLeft = NULL;
				}
				if (ImageRight != NULL)
				{
					free(ImageRight);
					ImageRight = NULL;
				}
				if (ImageColor != NULL)
				{
					free(ImageColor);
					ImageColor = NULL;
				}
				ImageLeft = new char[graywidth*grayheight];
				ImageRight = new char[graywidth*grayheight];
				ImageColor = new char[colorwidth*colorheight * 2];
				std::cout << "按回车键继续!" << endl;
				std::cin.getline(buf, 10);


			}
			else {
				std::cout << "连接设备错误，请确认设备已连接！" << endl;
				std::cout << "按回车键继续!" << endl;

			}
		}break;
		case 1:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_LASER_ON);
			break;
		case 2:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_LASER_OFF);
			break;
		case 3:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_IRLED_ON);
			break;
		case 4:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_IRLED_OFF);
			break;
		case 5:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_COLLISION_ENABLE);
			break;
		case 6:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_COLLISION_DISABLE);
			break;
		case 7:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_COLLISION_INFO_CLEAR);
			break;
		case 8:
			rlt = Aim_SetCollisinoDetectLevel(aimHandle, EI, 1);
			break;
		case 9:
			rlt = Aim_SetCollisinoDetectLevel(aimHandle, EI, 10);
			break;

		case 10:
			rlt = Aim_SetAcquireData(aimHandle, EI, DT_NONE);
			break;
		case 11:
			rlt = Aim_SetAcquireData(aimHandle, EI, DT_INFO);
			break;
		case 12:
			rlt = Aim_SetAcquireData(aimHandle, EI, DT_IMGDUAL);
			break;
		case 13:
			rlt = Aim_SetAcquireData(aimHandle, EI, DT_IMGCOLOR);
			break;
		case 14:
			rlt = Aim_SetAcquireData(aimHandle, EI, DT_INFO_IMGDUAL);
			break;
		case 15:
			rlt = Aim_SetAcquireData(aimHandle, EI, DT_INFO_IMGCOLOR);
			break;
		case 16:
			rlt = Aim_SetAcquireData(aimHandle, EI, DT_INFO_IMGDUAL_IMGCOLOR);
			break;
		case 17:
			rlt = Aim_SetAcquireData(aimHandle, EI, DT_MARKER_INFO_WITH_WIFI);//仅wifi时有效
			break;
		case 18:
			rlt = Aim_SetAcquireData(aimHandle, EI, DT_STATUS_INFO);//仅wifi时有效
			break;

		case 20:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_LCD_OFF);
			break;
		case 21:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_LCD_ON);
			break;
		case 23:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_LCD_PAGE_SUBPIXEL);
			break;
		case 25:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_LCD_PAGE_COLOR);
			break;
		case 26:
			rlt = Aim_SetLCDShowRawPoint(aimHandle, EI, true);
			break;
		case 27:
			rlt = Aim_SetLCDShowRawPoint(aimHandle, EI, false);
			break;

		case 30:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_AF_EXP_AUTO_ON);
			break;
		case 31:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_AF_EXP_AUTO_OFF);
			break;
		case 32:
			rlt = Aim_SetColorExpTime(aimHandle, EI, 4000);
			break;
		case 33:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_AF_CONTINUOUSLY_ON);
			break;
		case 34:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_AF_CONTINUOUSLY_OFF);
			break;
		case 35:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_AF_SINGLE);
			break;
		case 36:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_AF_FIX_INFINITY);
			break;

		case 40:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_DUALCAM_AUTO_EXP_ON);
			break;
		case 41:
			rlt = Aim_SetSystemCommand(aimHandle, EI, SC_DUALCAM_AUTO_EXP_OFF);
			break;
		case 42:
			rlt = Aim_SetDualExpTime(aimHandle, EI, 1000);
			break;
		case 43:
		{
#if GET_DATA_FROM_HARDWARE 
			Aim_SetAcquireData(aimHandle, EI, DT_NONE);
#else
			Aim_SetAcquireData(aimHandle, EI, DT_INFO);
#endif
			if (rlt != AIMOOE_OK) break;

#if GET_DATA_FROM_HARDWARE 
			rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
			rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
			if (rlt != AIMOOE_OK) break;
			if (markerSt.MarkerNumber == 0)break;
			vector<double> ZDis;
			for (int i = 0; i < markerSt.MarkerNumber; i++)
				ZDis.push_back(markerSt.MarkerCoordinate[i * 3 + 2]);
			sort(ZDis.begin(), ZDis.end(), greater<double>());
			int mid = ZDis.size() / 2;
			double dis = ZDis.at(mid);
			rlt = Aim_SetDualExpTimeByDistance(aimHandle, EI, dis);
			break;
		}
		case 44:
			rlt = Aim_SetFlashDelay(aimHandle, EI, 1, 0);
			break;

		case 50:
#if GET_DATA_FROM_HARDWARE 
			rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
			rlt = Aim_GetStatusInfo(aimHandle, EI, statusSt);
#endif
			if (rlt == AIMOOE_OK)
			{
				std::cout << "当前碰撞状态（0：未碰撞，1：检测到碰撞，2，未开启检测）：" << statusSt.CollisionStatus << endl;
				std::cout << "CPU温度(℃)：" << statusSt.Tcpu << "  " << "主板温度(℃)：" << statusSt.Tpcb << endl;
				std::cout << "硬件状态是否正常(1:正常，0：不正常)：" << (statusSt.HardwareStatus == HW_OK) << endl;
				std::cout << "帧率" << "   左相机：" << (int)(statusSt.LeftCamFps) << "   右相机：" << (int)(statusSt.RightCamFps)
					<< "   彩色相机：" << (int)(statusSt.ColorCamFps) << "   触控屏：" << (int)(statusSt.LCDFps) << endl;
				std::cout << "曝光时间" << "   左相机：" << statusSt.ExposureTimeLeftCam << "   右相机：" << statusSt.ExposureTimeRightCam << endl;
			}
			else if (rlt == AIMOOE_NOT_REFLASH)
			{
				std::cout << "请确认所要获取的数据已通过Aim_SetAcquireData函数正确设置并重试!" << endl;
			}
			break;
		case 51:
			do
			{
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
				if (rlt == AIMOOE_NOT_REFLASH)
					Sleep(10);
			} while (rlt == AIMOOE_NOT_REFLASH);
			if (rlt == AIMOOE_OK)
			{
				int markerNum = markerSt.MarkerNumber;
				std::cout << "开机后第" << markerSt.ID << "次采集" << endl;
				std::cout << "标记点背景光(1:正常，0：不正常)：" << (markerSt.MarkerBGLightStatus == BG_LIGHT_OK) << endl;
				std::cout << "检测到标记点数量：" << markerNum << endl;
				for (int i = 0; i < markerNum; i++)
				{
					std::cout << i << ":(" << markerSt.MarkerCoordinate[i * 3 + 0] << ", " << markerSt.MarkerCoordinate[i * 3 + 1] << ", " << markerSt.MarkerCoordinate[i * 3 + 2] << ")" << endl;
				}

				if (markerSt.PhantomMarkerGroupNumber > 0)//存在幻影点
				{
					//根据幻影点的总分组数初始化每组幻影点的存放空间
					vector<int> *PantomMarkerID = new vector<int>[markerSt.PhantomMarkerGroupNumber];
					for (int j = 0; j < markerSt.PhantomMarkerGroupNumber; j++)
					{
						PantomMarkerID[j].reserve(50);//每组预留50个点，理论最大值为200，但基本不可能达到。
					}

					//遍历幻影点警示数组，把幻影点归类到对应分组中
					for (int j = 0; j < markerNum; j++)
					{
						int WarningValue = markerSt.PhantomMarkerWarning[j];
						if (WarningValue> 0)//表示第j个点为幻影点
						{
							PantomMarkerID[WarningValue - 1].push_back(j);//把该点归类到对应的幻影点分组
						}
					}

					//输出
					std::cout << "可能存在的幻影点共有" << markerSt.PhantomMarkerGroupNumber << "组。" << endl;
					for (int j = 0; j < markerSt.PhantomMarkerGroupNumber; j++)
					{
						std::cout << "属于第" << j + 1 << "组的点序号：";
						for (int i = 0; i < PantomMarkerID[j].size(); i++)
						{
							std::cout << PantomMarkerID[j][i] << "  ";
						}
						std::cout << endl;
					}
					delete[] PantomMarkerID;
				}
				else
				{
					std::cout << "本次数据中不含幻影点" << endl;
				}
			}
			else if (rlt == AIMOOE_NOT_REFLASH)
			{
				std::cout << "请确认所要获取的数据已通过Aim_SetAcquireData函数正确设置并重试!" << endl;
			}
			break;
		case 52:
#if GET_DATA_FROM_HARDWARE 
			rlt = Aim_GetMarkerStatusAndGreyImageFromHardware(aimHandle, EI, markerSt, statusSt, ImageLeft, ImageRight);
#else
			rlt = Aim_GetGreyImageDual(aimHandle, EI, ImageLeft, ImageRight);
#endif		


			if (rlt == AIMOOE_OK)
			{
				if (saveToBmp((unsigned char*)ImageLeft, graywidth, grayheight, "./CameraImage/Left.bmp", BMP8BIT)
					&& saveToBmp((unsigned char*)ImageRight, graywidth, grayheight, "./CameraImage/Right.bmp", BMP8BIT))
					std::cout << "保存到./CameraImage/Left.bmp" << "和./CameraImage/Right.bmp" << endl;
				else
					std::cout << "保存失败" << endl;
			}
			else if (rlt == AIMOOE_NOT_REFLASH)
			{
				std::cout << "请确认所要获取的数据已通过Aim_SetAcquireData函数正确设置并重试!" << endl;
			}
			break;
		case 53:
#if GET_DATA_FROM_HARDWARE 
			rlt = Aim_GetColorImageFromHardware(aimHandle, EI, ImageColor);
#else
			rlt = Aim_GetColorImageMiddle(aimHandle, EI, ImageColor);
#endif	
			if (rlt == AIMOOE_OK)
			{
				if (saveToBmp((unsigned char*)ImageColor, colorwidth, colorheight, "./CameraImage/Color.bmp", BMP16BIT))
					std::cout << "保存到./CameraImage/Color.bmp" << endl;
				else
					std::cout << "保存失败" << endl;
			}
			else if (rlt == AIMOOE_NOT_REFLASH)
			{
				std::cout << "请确认所要获取的数据已通过Aim_SetAcquireData函数正确设置并重试!" << endl;
			}
			break;
		case	54://获取当前使用的IP地址信息
		{
			UCHAR a = 0, b = 0, c = 0, d = 0;
			rlt = Aim_GetAimPositionIP(aimHandle, EI, a, b, c, d);
			if (rlt == AIMOOE_OK)
			{
				string IP = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
				std::cout << "当前IP: " << IP << endl;
			}
			else
			{
				std::cout << "请先连接定位仪！" << endl;
			}
		}
		break;
		case 55:
			rlt = Aim_GetManufactureInfo(aimHandle, EI, manufactureInfo);
			strversion = string(manufactureInfo.Version, manufactureInfo.VersionLength);
			std::cout << "版本号:" << strversion.c_str() << endl;
			std::cout << "出厂日期：" << (int)manufactureInfo.Year << "/" << (int)manufactureInfo.Month << "/" << (int)manufactureInfo.Day << endl;
			break;
		case 56:
		{
			char addr[18];
			rlt = Aim_GetAimPositonMacAddress(aimHandle, addr);
			std::cout << "Mac地址:" << addr << endl;
			break;
		}
		case 60://获取某路径下的定位工具文件，并获取工具的信息
		{
			rlt = Aim_SetToolInfoFilePath(aimHandle, path);
			std::cout << Aim_GetToolInfoFilePath(aimHandle) << endl;

			int size = 0;
			Aim_GetCountOfToolInfo(aimHandle, size);
			std::cout << "路径下的工具文件数量为：" << size << endl;
			if (size != 0)
			{
				t_ToolBaseInfo *toolarr = new t_ToolBaseInfo[size];

				rlt = Aim_GetAllToolFilesBaseInfo(aimHandle, toolarr);

				if (rlt == AIMOOE_OK)
				{
					for (int i = 0; i < size; i++)
					{
						char *	ptool = toolarr[i].name;
						std::cout << "当前目录下的工具识别文件有：" << ptool << endl;
					}
				}
				delete[] toolarr;
			}
			else {
				std::cout << "当前目录没有工具识别文件：" << endl;
				break;
			}
			do
			{
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
				if (rlt == AIMOOE_NOT_REFLASH)
					Sleep(10);
			} while (rlt == AIMOOE_NOT_REFLASH);
			if (rlt != AIMOOE_OK)
				break;
			std::vector<int> residualIndex;//用于存放离散点的index
			for (int i = 0; i < markerSt.MarkerNumber; i++)
			{
				residualIndex.push_back(i);
			}
			T_AimToolDataResult *mtoolsrlt = new T_AimToolDataResult;
			mtoolsrlt->next = NULL;
			mtoolsrlt->validflag = false;
			rlt = Aim_FindToolInfo(aimHandle, markerSt, mtoolsrlt, 0);
			T_AimToolDataResult *prlt = mtoolsrlt;
			if (rlt == AIMOOE_OK)
			{
				do
				{
					if (prlt->validflag)
					{
						float PI = 3.141592;
						std::cout << "找到工具: " << prlt->toolname << "  平均误差" << prlt->MeanError << " RMS误差" << prlt->Rms << endl;
						std::cout << "工具原点:" << prlt->OriginCoor[0] << "," << prlt->OriginCoor[1] << "," << prlt->OriginCoor[2] << endl;
						std::cout << "工具角度:" << prlt->rotationvector[0] * 180 / PI << "," << prlt->rotationvector[1] * 180 / PI << "," << prlt->rotationvector[2] * 180 / PI << endl;
						std::cout << "标记点坐标:" << endl;
						for (int i = 0; i < prlt->toolptidx.size(); i++)
						{
							int idx = prlt->toolptidx[i];
							std::vector<int>::iterator iter = residualIndex.begin();
							iter = find(residualIndex.begin(), residualIndex.end(), idx);
							if (iter != residualIndex.end())
							{
								residualIndex.erase(iter);
							}
							if (idx < 0)
								cout << "0 0 0" << endl;
							else
								cout << markerSt.MarkerCoordinate[idx * 3 + 0] << " " << markerSt.MarkerCoordinate[idx * 3 + 1] << " " << markerSt.MarkerCoordinate[idx * 3 + 2] << " " << endl;
						}
					}
					T_AimToolDataResult *pnext = prlt->next;
					delete prlt;
					prlt = pnext;
				} while (prlt != NULL);
				cout << endl;
				if (residualIndex.size() > 0)
				{
					cout << "共" << residualIndex.size() << "个离散点：" << endl;
					for (int i = 0; i < residualIndex.size(); i++)
					{
						int j = residualIndex[i];
						cout << i << ":" << markerSt.MarkerCoordinate[j * 3 + 0] << "," << markerSt.MarkerCoordinate[j * 3 + 1] << "," << markerSt.MarkerCoordinate[j * 3 + 2] << endl;
					}
				}
			}
			else
			{
				delete prlt;
			}
			std::cout << "查找结束" << endl;
			rlt = AIMOOE_OK;
		}break;

		case 61://获取某特定的工具的具体信息
		{
			rlt = Aim_SetToolInfoFilePath(aimHandle, path);
			std::cout << Aim_GetToolInfoFilePath(aimHandle) << endl;
			std::vector<std::string>toolidarr;
			toolidarr.push_back("PT4M-CST005L");
			toolidarr.push_back("newTool");//请根据自行的需要，修改，增加或删减vector中的内容
			do
			{
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
				if (rlt == AIMOOE_NOT_REFLASH)
					Sleep(10);
			} while (rlt == AIMOOE_NOT_REFLASH);
			if (rlt != AIMOOE_OK)
				break;

			std::vector<int> residualIndex;//用于存放离散点的index
			for (int i = 0; i < markerSt.MarkerNumber; i++)
			{
				residualIndex.push_back(i);
			}
			T_AimToolDataResult *mtoolsrlt = new T_AimToolDataResult;
			mtoolsrlt->next = NULL;
			rlt = Aim_FindSpecificToolInfo(aimHandle, markerSt, toolidarr, mtoolsrlt, 3);
			T_AimToolDataResult *prlt = mtoolsrlt;
			if (rlt == AIMOOE_OK)
			{
				do
				{
					if (prlt->validflag)
					{
						float PI = 3.141592;
						std::cout << "找到工具: " << prlt->toolname << "  平均误差" << prlt->MeanError << " RMS误差" << prlt->Rms << endl;
						std::cout << "工具原点:" << prlt->OriginCoor[0] << "," << prlt->OriginCoor[1] << "," << prlt->OriginCoor[2] << endl;
						std::cout << "工具角度:" << prlt->rotationvector[0] * 180 / PI << "," << prlt->rotationvector[1] * 180 / PI << "," << prlt->rotationvector[2] * 180 / PI << endl;
						std::cout << "标记点坐标:" << endl;
						for (int i = 0; i < prlt->toolptidx.size(); i++)
						{
							int idx = prlt->toolptidx[i];
							std::vector<int>::iterator iter = residualIndex.begin();
							iter = find(residualIndex.begin(), residualIndex.end(), idx);
							if (iter != residualIndex.end())
							{
								residualIndex.erase(iter);
							}
							if (idx < 0)
								cout << "0 0 0" << endl;
							else
								cout << markerSt.MarkerCoordinate[idx * 3 + 0] << " " << markerSt.MarkerCoordinate[idx * 3 + 1] << " " << markerSt.MarkerCoordinate[idx * 3 + 2] << " " << endl;
						}
					}
					T_AimToolDataResult *pnext = prlt->next;
					delete prlt;
					prlt = pnext;
				} while (prlt != NULL);
				cout << endl;
				if (residualIndex.size() > 0)
				{
					cout << "共" << residualIndex.size() << "个离散点：" << endl;
					for (int i = 0; i < residualIndex.size(); i++)
					{
						int j = residualIndex[i];
						cout << i << ":" << markerSt.MarkerCoordinate[j * 3 + 0] << "," << markerSt.MarkerCoordinate[j * 3 + 1] << "," << markerSt.MarkerCoordinate[j * 3 + 2] << endl;
					}
				}
			}
			else
			{
				delete prlt;
			}
			std::cout << "查找结束" << endl;
			rlt = AIMOOE_OK;
		}
		break;

		case 62://空间配准
		{
			//假设从CT图像空间（或其他坐标系空间）中提取到了8个标记点坐标
			float CTMarkerPoint[8][3] = {
				16.343207381087609,23.593091198186006,896.24854687264269,
				83.067455320391687,49.586236895379429,915.91516659714512,
				-53.411448728043936,45.097685909857674,881.87007106341002,
				105.42002726073916,24.532249976922738,917.98572414341686,
				2.9486881579692295,69.657863821569578,907.50023478423850,
				102.56899569028074,79.047038309285625,930.24221029735918,
				147.27981734236465,82.039986372897090,931.68557520709078,
				30.759154671344831,91.715470291849613,927.82799622679840
			};
			//定义CT图像空间（或其他坐标系空间）中的目标点坐标，此处为方便测试，选择CT图像中的其中一个标记点
			float CTDstPoint[3] = { 16.343207381087609,23.593091198186006,896.24854687264269 };
			//CT图像空间的点集初始化
			rlt = Aim_InitMappingPointSetsForMarkerSpaceReg(aimHandle, CTMarkerPoint, 8);
			if (rlt != AIMOOE_OK)
				break;
			//定位系统获取实时的标记点坐标
			do
			{
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
				if (rlt == AIMOOE_NOT_REFLASH)
					Sleep(10);
			} while (rlt == AIMOOE_NOT_REFLASH);
			if (rlt != AIMOOE_OK)
				break;
			T_AimToolDataResult *prlt = new T_AimToolDataResult;
			prlt->next = NULL;
			prlt->validflag = false;
			//用实时的标记点坐标与CT图像标记点坐标进行空间配准
			rlt = Aim_MappingPointSetsForMarkerSpaceReg(aimHandle, markerSt, prlt, 3);
			if (rlt != AIMOOE_OK)
			{
				std::cout << "配准失败！" << endl;
				delete prlt;
				break;
			}
			if (prlt->validflag)//配准成功，相关的配准结果在prlt中
			{
				float PI = 3.141592;
				std::cout << "配准成功！ " << "  平均配准误差" << prlt->MeanError << endl;
				std::cout << "CT图像的标记点在光学定位系统中的坐标：" << endl;
				for (int i = 0; i < prlt->toolptidx.size(); i++)
				{
					int idx = prlt->toolptidx[i];
					if (idx < 0)
						std::cout << "0 0 0" << endl;
					else
						std::cout << markerSt.MarkerCoordinate[idx * 3 + 0] << " " << markerSt.MarkerCoordinate[idx * 3 + 1] << " " << markerSt.MarkerCoordinate[idx * 3 + 2] << " " << endl;
				}
				//用配准结果（RT）计算CT图像中目标点在光学定位系统中的坐标
				float RxCTDstPoint[3] = { 0,0,0 };//=Rto*CTDstPoint
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						RxCTDstPoint[i] += prlt->Rto[i][j] * CTDstPoint[j];
					}
				}
				float dstPointInOpticalSys[3];//=Rto*CTDstPoint+Tto
				for (int i = 0; i < 3; i++)
				{
					dstPointInOpticalSys[i] = RxCTDstPoint[i] + prlt->Tto[i];
				}
				std::cout << "CT图像的目标点在光学定位系统中的坐标：（" << dstPointInOpticalSys[0] << ", " << dstPointInOpticalSys[1] << ", " << dstPointInOpticalSys[2] << ")" << endl;

			}
			else
			{
				std::cout << "配准失败！" << endl;
			}
			delete prlt;

		}break;
		case 63://制作4点工具
		{
			t_ToolMadeProInfo mToolMadeinfo;
			rlt = Aim_SetToolInfoFilePath(aimHandle, path);
			std::cout << Aim_GetToolInfoFilePath(aimHandle) << endl;

			rlt = Aim_InitToolMadeInfo(aimHandle, 4, "newTool");
			if (rlt != AIMOOE_OK)
				break;
			int cntTimes = 0;
			do
			{
				Sleep(10);
				cntTimes++;
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
				if (rlt != AIMOOE_OK)
					continue;
				rlt = Aim_ProceedToolMade(aimHandle, markerSt, mToolMadeinfo);
				std::cout << "进度:" << mToolMadeinfo.madeRate * 100 << "%\r";
			} while (mToolMadeinfo.isMadeProFinished == false && cntTimes < 1000);
			std::cout << endl;
			if (mToolMadeinfo.isMadeProFinished)
			{
				Aim_SaveToolMadeRlt(aimHandle, true);
				std::cout << "保存工具文件：newTool.aimtool" << endl;
			}

			else
			{
				Aim_SaveToolMadeRlt(aimHandle, false);
				std::cout << "制作工具文件未达标，未保存" << endl;
			}

		}
		break;

		case 64://校准工具
		{
			t_ToolFixProInfo mToolFixinfo;
			rlt = Aim_SetToolInfoFilePath(aimHandle, path);
			std::cout << Aim_GetToolInfoFilePath(aimHandle) << endl;
			if (AIMOOE_OK != Aim_InitToolSelfCalibrationWithToolId(aimHandle, "newTool", mToolFixinfo.totalmarkcnt))
				break;
			int cntTimes = 0;
			do
			{
				Sleep(10);
				cntTimes++;
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
				if (rlt != AIMOOE_OK)
					continue;
				rlt = Aim_ProceedToolSelfCalibration(aimHandle, markerSt, mToolFixinfo);
				std::cout << "有效校准次数:" << mToolFixinfo.isValidFixCnt << "\r";
			} while (mToolFixinfo.isCalibrateFinished == false && cntTimes < 1000);
			std::cout << endl;
			if (mToolFixinfo.isCalibrateFinished && mToolFixinfo.MatchError<0.5)
			{
				Aim_SaveToolSelfCalibration(aimHandle, E_ToolFixRlt::eToolFixSave);
				std::cout << "工具校准完成，精度（mm）：" << mToolFixinfo.MatchError << endl;
			}
			else
			{
				Aim_SaveToolSelfCalibration(aimHandle, E_ToolFixRlt::eToolFixCancle);
				std::cout << "工具校准未成功" << endl;
			}

		}
		break;

		case 65://工具针尖注册（注册板）
		{
			t_ToolTipCalProInfo toolTipInfo;
			toolTipInfo.isCalibrateFinished = false;
			rlt = Aim_SetToolInfoFilePath(aimHandle, path);
			std::cout << Aim_GetToolInfoFilePath(aimHandle) << endl;

			rlt = Aim_InitToolTipCalibrationWithToolId(aimHandle, "CT4M-TA001", "newTool");
			if (rlt != AIMOOE_OK)break;
			int cntTimes = 0;
			do
			{
				Sleep(10);
				cntTimes++;
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
				if (rlt != AIMOOE_OK)
					continue;
				rlt = Aim_ProceedToolTipCalibration(aimHandle, markerSt, toolTipInfo);
				std::cout << "进度:" << (int)(toolTipInfo.CalibrateRate * 100) << "%\r";
			} while (toolTipInfo.isCalibrateFinished == false && cntTimes < 1000);
			std::cout << endl;
			if (toolTipInfo.isCalibrateFinished && toolTipInfo.CalRMSError < 0.5)
			{
				Aim_SaveToolTipCalibration(aimHandle);
				std::cout << "工具针尖注册完成，精度（mm）：" << toolTipInfo.CalRMSError << endl;
			}
			else
			{
				std::cout << "工具针尖注册未成功" << endl;
			}
		}
		break;
		case 75://工具针尖注册（绕点旋转）
		{
			rlt = Aim_SetToolInfoFilePath(aimHandle, path);
			std::cout << Aim_GetToolInfoFilePath(aimHandle) << endl;
			T_ToolTipPivotInfo toolTipInfo;
			toolTipInfo.isPivotFinished = false;
			rlt = Aim_InitToolTipPivotWithToolId(aimHandle, "newTool");
			if (rlt != AIMOOE_OK)break;
			int cntTimes = 0;
			do
			{
				Sleep(50);
				cntTimes++;
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
				if (rlt != AIMOOE_OK)
					continue;
				rlt = Aim_ProceedToolTipPivot(aimHandle, markerSt, toolTipInfo);
				std::cout << "进度:" << (int)(toolTipInfo.pivotRate * 100) << "%\r";
			} while (toolTipInfo.isPivotFinished == false && cntTimes < 1000);
			std::cout << endl;
			if (toolTipInfo.isPivotFinished && toolTipInfo.pivotMeanError < 1)
			{
				Aim_SaveToolTipCalibration(aimHandle);
				std::cout << "工具针尖注册完成，精度（mm）：" << toolTipInfo.pivotMeanError << endl;
			}
			else
			{
				std::cout << "工具针尖注册不成功，误差大于1mm" << endl;
			}
		}
		break;
		case 76://工具注册(坐标系转化)
		{
			t_ToolTipCalProInfo toolTipInfo;
			toolTipInfo.isCalibrateFinished = false;
			rlt = Aim_SetToolInfoFilePath(aimHandle, path);
			std::cout << Aim_GetToolInfoFilePath(aimHandle) << endl;
			rlt = Aim_InitToolCoordinateRenewWithToolId(aimHandle, "EDGE-CALIB", "PT4M-EDGE");//分别为注册板和工具文件的名称
			if (rlt != AIMOOE_OK)break;
			int cntTimes = 0;
			do
			{
				Sleep(10);
				cntTimes++;
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
				if (rlt != AIMOOE_OK)
					continue;
				rlt = Aim_ProceedToolCoordinateRenew(aimHandle, markerSt, toolTipInfo);
				std::cout << "进度:" << (int)(toolTipInfo.CalibrateRate) << "%" << '\r' << flush;
			} while (toolTipInfo.isCalibrateFinished == false && cntTimes < 1000);
			std::cout << endl;
			if (toolTipInfo.isCalibrateFinished && toolTipInfo.CalRMSError< 0.5)
			{
				Aim_SaveToolCoordinateRenew(aimHandle);
				std::cout << "削骨工具注册完成，精度（mm）：" << toolTipInfo.CalRMSError << endl;
			}
			else
			{
				std::cout << "削骨工具注册未成功" << endl;
			}
		}
		break;

		case 66://AAK精度工具测试
		{
			T_AccuracyToolResult accToolRlt;
			rlt = Aim_SetToolInfoFilePath(aimHandle, path);
			std::cout << Aim_GetToolInfoFilePath(aimHandle) << endl;
			rlt = Aim_InitAccuracyCheckTool(aimHandle, "AAS-B8CD1", "AAS-B4C1", "AAS-B4D1");
			if (rlt != AIMOOE_OK)
				break;
			for (int i = 0; i < 20; i++)
			{
				Sleep(100);
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif	
				if (rlt != AIMOOE_OK)
					continue;
				double **markerArr = new double *[markerSt.MarkerNumber];
				for (int j = 0; j < markerSt.MarkerNumber; j++)
				{
					markerArr[j] = new double[3];
				}
				int a = sizeof(markerArr);
				for (int j = 0; j < markerSt.MarkerNumber; j++)
				{
					markerArr[j][0] = markerSt.MarkerCoordinate[j * 3 + 0];
					markerArr[j][1] = markerSt.MarkerCoordinate[j * 3 + 1];
					markerArr[j][2] = markerSt.MarkerCoordinate[j * 3 + 2];
				}
				rlt = Aim_AccuracyCheckTool(aimHandle, markerArr, markerSt.MarkerNumber, accToolRlt);
				if (accToolRlt.validflag)
					cout << accToolRlt.Angle[0] << ", " << accToolRlt.Angle[1] << ", " << accToolRlt.Angle[2] << ", " << accToolRlt.Dis << endl;

				for (int j = 0; j < markerSt.MarkerNumber; j++)
				{
					delete[] markerArr[j];
				}
				delete[] markerArr;
			}
			float angle[3], meanerror, stdev;
			Aim_AccuracyCheckToolCalculateError(aimHandle, meanerror, stdev, angle);
			cout << "中心距离误差均值" << meanerror << "，中心距离误差离散性" << stdev << "，角度偏差均值：(" << angle[0]  << ", " << angle[1]  << ", " << angle[2] << ")" << endl;

		}
		break;
		case 67://检查工具文件信息
		{
			T_ToolFileData data;
			rlt = Aim_CheckToolFile(aimHandle, ".\\AimTools\\newTool.aimtool", data);
			cout << data.toolname << endl;
			cout << data.tooType << endl;
			cout << data.markerNumbers << endl;
			int num = data.markerNumbers;
			num = num > 16 ? 16 : num;
			for (int i = 0; i < num; i++)
				cout << data.MarkerCoordinate[i * 3 + 0] << ", " << data.MarkerCoordinate[i * 3 + 1] << ", " << data.MarkerCoordinate[i * 3 + 2] << endl;
			cout << data.tipHeadCoordinate[0] << ", " << data.tipHeadCoordinate[1] << ", " << data.tipHeadCoordinate[2] << endl;
			cout << data.tipBodyCoordinate[0] << ", " << data.tipBodyCoordinate[1] << ", " << data.tipBodyCoordinate[2] << endl;
			if (rlt == AIMOOE_OK)
				cout << "check ok!" << endl;
			else
				cout << "check failed!" << endl;
		}
		break;
		case 98:
			return 1;
			break;
		case 99:
		{
			if (ImageLeft != NULL)
			{
				delete[]ImageLeft;
				ImageLeft = NULL;
			}
			if (ImageRight != NULL)
			{
				delete[]ImageRight;
				ImageRight = NULL;
			}
			if (ImageColor != NULL)
			{
				delete[]ImageColor;
				ImageColor = NULL;
			}
			return 0;
		}

		break;
		default:
			number = 99;



			rlt = AIMOOE_OK;
			break;
		}

		if ((rlt != AIMOOE_OK) && (rlt != AIMOOE_NOT_REFLASH))
		{
			std::cout << "指令出错,请重启再试！" << endl;
			//return;
		}
		else if (number == 99)
		{
			std::cout << "指令号码有误，请重新输入！";
		}
		else
		{
			std::cout << "指令成功！" << endl;

		}
		std::cout << "按回车键继续!" << endl;
		std::cin.getline(buf, 10);
		std::cout << endl;
	}

}

void setText()
{
	std::cout << "请输入指令号码（00-99）并按回车键：" << endl;
	std::cout << "00.重新连接设备" << endl;
	std::cout << "01.打开激光灯" << endl;
	std::cout << "02.关闭激光灯" << endl;
	std::cout << "03.打开红外照明环" << endl;
	std::cout << "04.关闭红外照明环" << endl;
	std::cout << "05.打开碰撞检测" << endl;
	std::cout << "06.关闭碰撞检测" << endl;
	std::cout << "07.清除碰撞标志" << endl;
	std::cout << "08.设置碰撞检测灵敏度为1（最灵敏）" << endl;
	std::cout << "09.设置碰撞检测灵敏度为10（最不灵敏）" << endl;

	std::cout << "10.设置从定位仪获取的数据为空" << endl;
	std::cout << "11.设置从定位仪获取的数据为信息数据（含系统状态和三维坐标）" << endl;
	if (EI != I_WIFI)
	{
		std::cout << "12.设置从定位仪获取的数据为双目灰度图像数据" << endl;
		std::cout << "13.设置从定位仪获取的数据为中间彩色图像数据" << endl;
		std::cout << "14.设置从定位仪获取的数据为信息数据和双目灰度图像数据" << endl;
		std::cout << "15.设置从定位仪获取的数据为信息数据和中间彩色图像数据" << endl;
		std::cout << "16.设置从定位仪获取的数据为信息数据，双目灰度图像数据和中间彩色图像数据" << endl;
	}
	else
	{
		std::cout << "17.设置从定位仪获取的数据为三维坐标数据" << endl;
		std::cout << "18.设置从定位仪获取的数据为系统状态数据" << endl;
	}

	std::cout << "20.关闭触控显示屏" << endl;
	std::cout << "21.打开触控显示屏" << endl;
	std::cout << "22.复位触控显示屏" << endl;
	std::cout << "23.切换触控屏页面到：标记点检测页面" << endl;
	std::cout << "25.切换触控屏页面到：中间彩色图像页面" << endl;
	std::cout << "26.设置触控屏标记点检测页面显示的点为未经处理的点" << endl;
	std::cout << "27.设置触控屏标记点检测页面显示的点为处理后的点" << endl;

	std::cout << "30.启动中间相机自动曝光" << endl;
	std::cout << "31.关闭中间相机自动曝光" << endl;
	std::cout << "32.设置中间相机曝光值为4000" << endl;
	std::cout << "33.启动中间相机持续对焦" << endl;
	std::cout << "34.关闭中间相机持续对焦" << endl;
	std::cout << "35.启动中间相机单次对焦" << endl;
	std::cout << "36.中间相机固定焦距到无穷远" << endl;

	std::cout << "40.启动双目自动曝光(开机默认关闭)" << endl;
	std::cout << "41.关闭双目自动曝光(开机默认关闭)" << endl;
	std::cout << "42.设置双目相机曝光值（左右均为1000）" << endl;
	std::cout << "43.以采集到的标记点的Z轴值为距离设置曝光值（多个点时取中值）" << endl;
	std::cout << "44.设置闪光灯延时" << endl;
	std::cout << "50.获取系统状态信息" << endl;
	std::cout << "51.获取标记点三维坐标信息" << endl;
	if (EI != I_WIFI)
	{
		std::cout << "52.获取左右相机图像Left.bmp和Right.bmp到CameraImage文件夹" << endl;
		std::cout << "53.获取中间彩色相机图像Color.bmp到CameraImage文件夹" << endl;
	}
	std::cout << "54.获取当前使用的IP地址信息" << endl;
	std::cout << "55.获取出厂信息" << endl;
	std::cout << "56.获取设备Mac地址" << endl;
	std::cout << "60.获取某路径下的定位工具文件，并获取工具的信息" << endl;
	std::cout << "61.获取某特定的工具的具体信息" << endl;
	std::cout << "62.空间配准" << endl;
	std::cout << "63.制作4点工具" << endl;
	std::cout << "64.校准工具" << endl;
	std::cout << "65.工具针尖注册（注册板）" << endl;
	std::cout << "75.工具针尖注册（绕点旋转）" << endl;
	std::cout << "76.工具注册--坐标转换版本（把工具文件的点坐标转到注册版工具文件的坐标系上。）" << endl;
	std::cout << "66.AAK精度工具测试" << endl;
	std::cout << "67.检查工具文件" << endl;
	std::cout << "98.重新选择连接方式" << endl;
	std::cout << "99.退出" << endl;
}


void write_bmpheader(unsigned char *bitmap, int offset, int bytes, int value)
{
	int i;
	for (i = 0; i < bytes; i++)
		bitmap[offset + i] = (value >> (i << 3)) & 0xFF;
}

unsigned char *convertGrayTo24bitBmp(unsigned char *inputImg, int width, int height, int *ouputSize)
{
	/*create a bmp format file*/
	int bitmap_x = (int)ceil((double)width * 3 / 4) * 4;
	unsigned char *bitmap = (unsigned char*)malloc(sizeof(unsigned char)*height*bitmap_x + 54);

	bitmap[0] = 'B';
	bitmap[1] = 'M';
	write_bmpheader(bitmap, 2, 4, height*bitmap_x + 54); //whole file size
	write_bmpheader(bitmap, 0xA, 4, 54); //offset before bitmap raw data
	write_bmpheader(bitmap, 0xE, 4, 40); //length of bitmap info header
	write_bmpheader(bitmap, 0x12, 4, width); //width
	write_bmpheader(bitmap, 0x16, 4, height); //height
	write_bmpheader(bitmap, 0x1A, 2, 1);
	write_bmpheader(bitmap, 0x1C, 2, 24); //bit per pixel
	write_bmpheader(bitmap, 0x1E, 4, 0); //compression
	write_bmpheader(bitmap, 0x22, 4, height*bitmap_x); //size of bitmap raw data
	for (int i = 0x26; i < 0x36; i++)
		bitmap[i] = 0;

	int k = 54;

	for (int i = height - 1; i >= 0; i--) //bmp图像存储从最后一行开始
	{
		int j;
		for (j = 0; j < width; j++)
		{
			int index = i*width + j;
			bitmap[k++] = inputImg[index];
			bitmap[k++] = inputImg[index];
			bitmap[k++] = inputImg[index];
		}
		j *= 3;
		while (j < bitmap_x) {
			bitmap[k++] = 0;
			j++;
		}
	}

	*ouputSize = k;
	return bitmap;
}
unsigned char *convertColor16To24bitBmp(unsigned char *inputImg, int width, int height, int *ouputSize)
{
	/*create a bmp format file*/
	int bitmap_x = (int)ceil((double)width * 3 / 4) * 4;
	unsigned char *bitmap = (unsigned char*)malloc(sizeof(unsigned char)*height*bitmap_x + 54);

	bitmap[0] = 'B';
	bitmap[1] = 'M';
	write_bmpheader(bitmap, 2, 4, height*bitmap_x + 54); //whole file size
	write_bmpheader(bitmap, 0xA, 4, 54); //offset before bitmap raw data
	write_bmpheader(bitmap, 0xE, 4, 40); //length of bitmap info header
	write_bmpheader(bitmap, 0x12, 4, width); //width
	write_bmpheader(bitmap, 0x16, 4, height); //height
	write_bmpheader(bitmap, 0x1A, 2, 1);
	write_bmpheader(bitmap, 0x1C, 2, 24); //bit per pixel
	write_bmpheader(bitmap, 0x1E, 4, 0); //compression
	write_bmpheader(bitmap, 0x22, 4, height*bitmap_x); //size of bitmap raw data
	for (int i = 0x26; i < 0x36; i++)
		bitmap[i] = 0;

	int k = 54;

	for (int i = height - 1; i >= 0; i--) //bmp图像存储从最后一行开始
	{
		int j;
		for (j = 0; j < width; j++)
		{
			int index = (i*width + j) * 2;
			bitmap[k++] = (inputImg[index] << 3);//blue
			bitmap[k++] = ((inputImg[index + 1] << 5) | (inputImg[index] >> 3)) & 0xFC;//green
			bitmap[k++] = inputImg[index + 1] & 0xF8;//red
		}
		j *= 3;
		while (j < bitmap_x) {
			bitmap[k++] = 0;
			j++;
		}
	}

	*ouputSize = k;
	return bitmap;
}

bool saveToBmp(unsigned char *inputImg, int width, int height, char *outputFileName, E_BMPBIT bmpBit)
{
	int size;
	unsigned char *bmp;
	if (bmpBit == BMP8BIT)
		bmp = convertGrayTo24bitBmp(inputImg, width, height, &size);
	else
		bmp = convertColor16To24bitBmp(inputImg, width, height, &size);
	FILE *fp = fopen(outputFileName, "wb+");
	if (fp == NULL) {
		return false;
	}
	fwrite(bmp, 1, size, fp);
	fclose(fp);
	free(bmp);

	return true;
}

