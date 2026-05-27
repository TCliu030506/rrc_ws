/*
舵机出厂速度单位是0.0146rpm，速度改为V=2400
*/

#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;

int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
	}
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sm_st.begin(1000000, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }
	
	int Pos;
	int PosStart = 900;
	int PosEnd = 2800;
	// int IdealStep = rand()%(1500);
	int IdealStep = 200;
	int RealStepErr[1900];
	int PosCurrent;
	int StepErr;
	PosCurrent = PosStart+IdealStep;
	// while(PosCurrent<(PosEnd+1)){
	// 	sm_st.WritePosEx(1, PosCurrent, 500, 50);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
	// 	std::cout<<"pos_1 = "<<PosCurrent<<std::endl;
	// 	usleep(3*1000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
	// 	Pos = sm_st.ReadPos(1);
	// 	if(Pos!=-1){
	// 		std::cout<<"pos_1_actual = "<<Pos<<std::endl;
	// 		usleep(10*1000);
	// 	}else{
	// 		std::cout<<"read pos_1_actual err"<<std::endl;
	// 		sleep(1);
	// 	}
	// 	PosCurrent = PosCurrent+IdealStep;
	// 	RealStepErr[PosCurrent-PosStart] = Pos-PosCurrent;
	// 	std::cout<<"RealStepErr = "<<RealStepErr[PosCurrent-PosStart]<<std::endl;
	// }
		
	sm_st.WritePosEx(1, PosCurrent, 500, 50);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
	std::cout<<"pos_1_target = "<<PosCurrent<<std::endl;
	usleep(IdealStep*5*1000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
	Pos = sm_st.ReadPos(1);
	if(Pos!=-1){
		std::cout<<"pos_1_actual = "<<Pos<<std::endl;
	}else{
		std::cout<<"read pos_1_actual err"<<std::endl;
	}
	StepErr = Pos-PosCurrent;
	std::cout<<"StepErr = "<<StepErr<<std::endl;

	//如果步距误差比较大，则重新发送目标运动指令
	while(StepErr>2 or StepErr<-2){
		sm_st.WritePosEx(1, PosCurrent, 500, 50);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
		std::cout<<"重新发送:pos_1_target = "<<PosCurrent<<std::endl;
		usleep(20*1000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
		Pos = sm_st.ReadPos(1);
		if(Pos!=-1){
			std::cout<<"重新检测:pos_1_actual = "<<Pos<<std::endl;
		}else{
			std::cout<<"read pos_1_actual err"<<std::endl;
		}
		StepErr = Pos-PosCurrent;
		std::cout<<"重新检测:StepErr = "<<StepErr<<std::endl;
	}

	sm_st.WritePosEx(1, PosStart, 500, 50);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
	std::cout<<"回到原点:pos_1_target = "<<PosStart<<std::endl;
	usleep(IdealStep*4*1000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
	Pos = sm_st.ReadPos(1);
	if(Pos!=-1){
		std::cout<<"成功返回:pos_1_actual = "<<Pos<<std::endl;
	}else{
		std::cout<<"read pos_1_actual err"<<std::endl;
	}

	sm_st.end();
	return 1;
}

