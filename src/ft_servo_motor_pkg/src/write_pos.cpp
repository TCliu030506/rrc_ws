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
	while(1){
		double Pos;
		double PosStart = 900;
		double PosEnd = 1100;
		double acc = 50;
		double speed = 500;
		double move_time = ((PosEnd-PosStart)/speed)*1000+((speed/acc)/100)*1000;

		sm_st.WritePosEx(1, PosEnd, speed, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
		std::cout<<"pos_1 = "<<PosEnd<<std::endl;
		usleep(move_time*2000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
		Pos = sm_st.ReadPos(1);
		if(Pos!=-1){
			std::cout<<"pos = "<<Pos<<std::endl;
			usleep(10*1000);
		}else{
			std::cout<<"read pos err"<<std::endl;
			sleep(1);
		}
  
		sm_st.WritePosEx(1, PosStart, speed, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P0=0位置
		std::cout<<"pos_1 = "<<PosStart<<std::endl;
		usleep(move_time*2000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
		Pos = sm_st.ReadPos(1);
		if(Pos!=-1){
			std::cout<<"pos = "<<Pos<<std::endl;
			usleep(10*1000);
		}else{
			std::cout<<"read pos err"<<std::endl;
			sleep(1);
		}

		sm_st.WritePosEx(2, PosEnd, speed, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
		std::cout<<"pos_2 = "<<PosEnd<<std::endl;
		usleep(move_time*2000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
		Pos = sm_st.ReadPos(2);
		if(Pos!=-1){
			std::cout<<"pos = "<<Pos<<std::endl;
			usleep(10*1000);
		}else{
			std::cout<<"read pos err"<<std::endl;
			sleep(1);
		}
  
		sm_st.WritePosEx(2, PosStart, speed, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P0=0位置
		std::cout<<"pos_2 = "<<PosStart<<std::endl;
		usleep(move_time*2000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
		Pos = sm_st.ReadPos(2);
		if(Pos!=-1){
			std::cout<<"pos = "<<Pos<<std::endl;
			usleep(10*1000);
		}else{
			std::cout<<"read pos err"<<std::endl;
			sleep(1);
		}
		sm_st.WritePosEx(3, PosEnd, speed, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
		std::cout<<"pos_3 = "<<PosEnd<<std::endl;
		usleep(move_time*2000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
  		Pos = sm_st.ReadPos(3);
		if(Pos!=-1){
			std::cout<<"pos = "<<Pos<<std::endl;
			usleep(10*1000);
		}else{
			std::cout<<"read pos err"<<std::endl;
			sleep(1);
		}

		sm_st.WritePosEx(3, PosStart, speed, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P0=0位置
		std::cout<<"pos_3 = "<<PosStart<<std::endl;
		usleep(move_time*2000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
		Pos = sm_st.ReadPos(3);
		if(Pos!=-1){
			std::cout<<"pos = "<<Pos<<std::endl;
			usleep(10*1000);
		}else{
			std::cout<<"read pos err"<<std::endl;
			sleep(1);
		}
	}
	sm_st.end();
	return 1;
}

