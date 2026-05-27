#include "../include/pf_control/pf_control.h"

// 创建对象时：pf_control controller("/dev/ttyUSB0");
pf_control::pf_control(const std::string& port):serial_port(port){
    pf_serial_open();
};

pf_control::~pf_control()
{
    pf_speed_off();
};

void pf_control::pf_serial_open(){
    if(!sm_st.begin(1000000, serial_port.c_str())){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
    }
}

void pf_control::pf_movep(double x, double y, double z, double t){
	if (pf_kin.is_within_SafetyDomain(x,y,z))
	{
		double Pos1,Pos2,Pos3;
        double target_theta1,target_theta2,target_theta3;
        double Pos1_read,Pos2_read,Pos3_read;
		double acc = 50;
		double speed1,speed2,speed3;

        Pos1_read = sm_st.ReadPos(1);
        Pos2_read = sm_st.ReadPos(2);
        Pos3_read = sm_st.ReadPos(3);

        double theta1 = (Pos1_read-900)*FT_STEP/180*M_PI*5.0/10.5;
        double theta2 = (Pos2_read-900)*FT_STEP/180*M_PI*5.0/10.5;
        double theta3 = (Pos3_read-900)*FT_STEP/180*M_PI*5.0/10.5;

        pf_kin.PF_inverse_kinematics(theta1,theta2,theta3,x,y,z,target_theta1,target_theta2,target_theta3);

		if(pf_kin.is_within_SoftLimit(target_theta1,target_theta2,target_theta3))
		{
			Pos1 = target_theta1/M_PI*180 *10.5/5.0 /FT_STEP+900;
			Pos2 = target_theta2/M_PI*180 *10.5/5.0 /FT_STEP+900;
			Pos3 = target_theta3/M_PI*180 *10.5/5.0 /FT_STEP+900;
			if( (Pos1>900 && Pos1<3060)&&(Pos2>900 && Pos2<3060)&&(Pos3>900 && Pos3<3060))
			{
				speed1 = std::abs(Pos1-Pos1_read)/t;
				speed2 = std::abs(Pos2-Pos2_read)/t;
				speed3 = std::abs(Pos3-Pos3_read)/t;

				sm_st.WritePosEx(1, Pos1, speed1, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos1<<std::endl;
				sm_st.WritePosEx(2, Pos2, speed2, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos2<<std::endl;
				sm_st.WritePosEx(3, Pos3, speed3, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos3<<std::endl;

				while((sm_st.ReadMove(1))||(sm_st.ReadMove(2))||(sm_st.ReadMove(3))){
					usleep(0.2*1000);
				};

				Pos1_read = sm_st.ReadPos(1);
				if(Pos1_read!=-1){
					std::cout<<"pos1_current = "<<Pos1_read<<std::endl;
					// usleep(0.05*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}

				Pos2_read = sm_st.ReadPos(2);
				if(Pos2_read!=-1){
					std::cout<<"pos2_current = "<<Pos2_read<<std::endl;
					// usleep(0.05*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}

				Pos3_read = sm_st.ReadPos(3);
				if(Pos3_read!=-1){
					std::cout<<"pos3_current = "<<Pos3_read<<std::endl;
					// usleep(0.5*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}
			}else{
				std::cout<<"错误：某一角度不在可行范围内！"<<std::endl;
			}
		}
        
	}else{
		std::cout<<"错误：目标点不在安全域内！"<<std::endl;
	}
}

bool pf_control::pf_movep_withflag(double x, double y, double z, double t){
	if (pf_kin.is_within_SafetyDomain(x,y,z))
	{
		double Pos1,Pos2,Pos3;
        double target_theta1,target_theta2,target_theta3;
        double Pos1_read,Pos2_read,Pos3_read;
		double acc = 50;
		double speed1,speed2,speed3;

        Pos1_read = sm_st.ReadPos(1);
        Pos2_read = sm_st.ReadPos(2);
        Pos3_read = sm_st.ReadPos(3);

        double theta1 = (Pos1_read-900)*FT_STEP/180*M_PI*5.0/10.5;
        double theta2 = (Pos2_read-900)*FT_STEP/180*M_PI*5.0/10.5;
        double theta3 = (Pos3_read-900)*FT_STEP/180*M_PI*5.0/10.5;

        pf_kin.PF_inverse_kinematics(theta1,theta2,theta3,x,y,z,target_theta1,target_theta2,target_theta3);

		if(pf_kin.is_within_SoftLimit(target_theta1,target_theta2,target_theta3))
		{
			Pos1 = target_theta1/M_PI*180 *10.5/5.0 /FT_STEP+900;
			Pos2 = target_theta2/M_PI*180 *10.5/5.0 /FT_STEP+900;
			Pos3 = target_theta3/M_PI*180 *10.5/5.0 /FT_STEP+900;
			if( (Pos1>900 && Pos1<3060)&&(Pos2>900 && Pos2<3060)&&(Pos3>900 && Pos3<3060))
			{
				speed1 = std::abs(Pos1-Pos1_read)/t;
				speed2 = std::abs(Pos2-Pos2_read)/t;
				speed3 = std::abs(Pos3-Pos3_read)/t;

				sm_st.WritePosEx(1, Pos1, speed1, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos1<<std::endl;
				sm_st.WritePosEx(2, Pos2, speed2, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos2<<std::endl;
				sm_st.WritePosEx(3, Pos3, speed3, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos3<<std::endl;

				while((sm_st.ReadMove(1))||(sm_st.ReadMove(2))||(sm_st.ReadMove(3))){
					usleep(0.2*1000);
				};

				Pos1_read = sm_st.ReadPos(1);
				if(Pos1_read!=-1){
					std::cout<<"pos1_current = "<<Pos1_read<<std::endl;
					// usleep(0.05*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}

				Pos2_read = sm_st.ReadPos(2);
				if(Pos2_read!=-1){
					std::cout<<"pos2_current = "<<Pos2_read<<std::endl;
					// usleep(0.05*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}

				Pos3_read = sm_st.ReadPos(3);
				if(Pos3_read!=-1){
					std::cout<<"pos3_current = "<<Pos3_read<<std::endl;
					// usleep(0.5*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}
				return true;
			}else{
				std::cout<<"错误：某一角度不在可行范围内！"<<std::endl;
				return false;
			}
		}
		else{
			std::cout<<"错误：目标点不在软限制范围内！"<<std::endl;
			return false;
		}
        
	}else{
		std::cout<<"错误：目标点不在安全域内！"<<std::endl;
		return false;
	}
}

void pf_control::pf_movep_corrected(double x, double y, double z, double t){
	// 尝试以实际装配的位置为原点
	x = x - initial_pos[0];
	y = y - initial_pos[1];
	z = z - initial_pos[2];
	if (pf_kin.is_within_SafetyDomain(x,y,z))
	{
		double Pos1,Pos2,Pos3;
        double target_theta1,target_theta2,target_theta3;
        double Pos1_read,Pos2_read,Pos3_read;
		double acc = 50;
		double speed1,speed2,speed3;

        Pos1_read = sm_st.ReadPos(1);
        Pos2_read = sm_st.ReadPos(2);
        Pos3_read = sm_st.ReadPos(3);

        double theta1 = (Pos1_read-900)*FT_STEP/180*M_PI*5.0/10.5;
        double theta2 = (Pos2_read-900)*FT_STEP/180*M_PI*5.0/10.5;
        double theta3 = (Pos3_read-900)*FT_STEP/180*M_PI*5.0/10.5;

        pf_kin.PF_inverse_kinematics(theta1,theta2,theta3,x,y,z,target_theta1,target_theta2,target_theta3);

		if(pf_kin.is_within_SoftLimit(target_theta1,target_theta2,target_theta3))
		{
			Pos1 = target_theta1/M_PI*180 *10.5/5.0 /FT_STEP+900;
			Pos2 = target_theta2/M_PI*180 *10.5/5.0 /FT_STEP+900;
			Pos3 = target_theta3/M_PI*180 *10.5/5.0 /FT_STEP+900;
			if( (Pos1>900 && Pos1<3060)&&(Pos2>900 && Pos2<3060)&&(Pos3>900 && Pos3<3060))
			{
				speed1 = std::abs(Pos1-Pos1_read)/t;
				speed2 = std::abs(Pos2-Pos2_read)/t;
				speed3 = std::abs(Pos3-Pos3_read)/t;

				sm_st.WritePosEx(1, Pos1, speed1, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos1<<std::endl;
				sm_st.WritePosEx(2, Pos2, speed2, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos2<<std::endl;
				sm_st.WritePosEx(3, Pos3, speed3, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos3<<std::endl;

				while((sm_st.ReadMove(1))||(sm_st.ReadMove(2))||(sm_st.ReadMove(3))){
					usleep(0.2*1000);
				};

				Pos1_read = sm_st.ReadPos(1);
				if(Pos1_read!=-1){
					std::cout<<"pos1_current = "<<Pos1_read<<std::endl;
					// usleep(0.05*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}

				Pos2_read = sm_st.ReadPos(2);
				if(Pos2_read!=-1){
					std::cout<<"pos2_current = "<<Pos2_read<<std::endl;
					// usleep(0.05*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}

				Pos3_read = sm_st.ReadPos(3);
				if(Pos3_read!=-1){
					std::cout<<"pos3_current = "<<Pos3_read<<std::endl;
					// usleep(0.5*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}
			}else{
				std::cout<<"错误：某一角度不在可行范围内！"<<std::endl;
			}
		}
        
	}else{
		std::cout<<"错误：目标点不在安全域内！"<<std::endl;
	}
}

void pf_control::pf_movep(EV3d pos, EV3d vel){
	double x = pos[0];
	double y = pos[1];
	double z = pos[2];
	double vel_x = vel[0];
	double vel_y = vel[1];
	double vel_z = vel[2];

	if (pf_kin.is_within_SafetyDomain(x,y,z))
	{
		double Pos1,Pos2,Pos3;
        double target_theta1,target_theta2,target_theta3;
        double Pos1_read,Pos2_read,Pos3_read;

		double speed1,speed2,speed3;
		EV3d target_speed = EV3d(speed1,speed2,speed3);

        Pos1_read = sm_st.ReadPos(1);
        Pos2_read = sm_st.ReadPos(2);
        Pos3_read = sm_st.ReadPos(3);

        double theta1 = (Pos1_read-900)*FT_STEP/180*M_PI*5.0/10.5;
        double theta2 = (Pos2_read-900)*FT_STEP/180*M_PI*5.0/10.5;
        double theta3 = (Pos3_read-900)*FT_STEP/180*M_PI*5.0/10.5;

        pf_kin.PF_inverse_kinematics(theta1,theta2,theta3,x,y,z,target_theta1,target_theta2,target_theta3);
		
		if(pf_kin.is_within_SoftLimit(target_theta1,target_theta2,target_theta3))
		{
			Pos1 = target_theta1/M_PI*180 *10.5/5.0 /FT_STEP+900;
			Pos2 = target_theta2/M_PI*180 *10.5/5.0 /FT_STEP+900;
			Pos3 = target_theta3/M_PI*180 *10.5/5.0 /FT_STEP+900;

			pf_kin.PF_inverse_vel(EV3d(theta1,theta2,theta3),EV3d(vel_x,vel_y,vel_z),0.001,target_speed);
			speed1 = target_speed[0]/M_PI*180 *10.5/5.0 /FT_STEP;
			speed2 = target_speed[1]/M_PI*180 *10.5/5.0 /FT_STEP;
			speed3 = target_speed[2]/M_PI*180 *10.5/5.0 /FT_STEP;

			double acc_1 = speed1;
			double acc_2 = speed2;
			double acc_3 = speed3;

			if( (Pos1>900 && Pos1<3060)&&(Pos2>900 && Pos2<3060)&&(Pos3>900 && Pos3<3060))
			{

				sm_st.WritePosEx(1, Pos1, speed1, acc_1);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos1<<std::endl;
				sm_st.WritePosEx(2, Pos2, speed2, acc_2);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos2<<std::endl;
				sm_st.WritePosEx(3, Pos3, speed3, acc_3);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos3<<std::endl;

				while((sm_st.ReadMove(1))||(sm_st.ReadMove(2))||(sm_st.ReadMove(3))){
					usleep(0.02*1000);
				};

				Pos1_read = sm_st.ReadPos(1);
				if(Pos1_read!=-1){
					std::cout<<"pos1_current = "<<Pos1_read<<std::endl;
					// usleep(0.05*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}

				Pos2_read = sm_st.ReadPos(2);
				if(Pos2_read!=-1){
					std::cout<<"pos2_current = "<<Pos2_read<<std::endl;
					// usleep(0.05*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}

				Pos3_read = sm_st.ReadPos(3);
				if(Pos3_read!=-1){
					std::cout<<"pos3_current = "<<Pos3_read<<std::endl;
					// usleep(0.5*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}
			}else{
				std::cout<<"错误：某一角度不在可行范围内！"<<std::endl;
			}
		}
        
	}else{
		std::cout<<"错误：目标点不在安全域内！"<<std::endl;
	}
}

void pf_control::pf_movej(double theta1, double theta2, double theta3, double t){

		double acc = 50;
		double speed1,speed2,speed3 ;
		double Pos1_read,Pos2_read,Pos3_read;

		if(pf_kin.is_within_SoftLimit(theta1/180.0*M_PI,theta2/180.0*M_PI,theta3/180.0*M_PI))
		{
			double Pos1 = theta1 *10.5/5.0 /FT_STEP+900;
			double Pos2 = theta2 *10.5/5.0 /FT_STEP+900;
			double Pos3 = theta3 *10.5/5.0 /FT_STEP+900;
	
			if( (Pos1>900 && Pos1<3060)&&(Pos2>900 && Pos2<3060)&&(Pos3>900 && Pos3<3060))
			{
				Pos1_read = sm_st.ReadPos(1);
				Pos2_read = sm_st.ReadPos(2);
				Pos3_read = sm_st.ReadPos(3);
	
				speed1 = std::abs(Pos1-Pos1_read)/t;
				speed2 = std::abs(Pos2-Pos2_read)/t;
				speed3 = std::abs(Pos3-Pos3_read)/t;
	
				sm_st.WritePosEx(1, Pos1, speed1, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos1<<std::endl;
				sm_st.WritePosEx(2, Pos2, speed2, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos2<<std::endl;
				sm_st.WritePosEx(3, Pos3, speed3, acc);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
				std::cout<<"pos_1 = "<<Pos3<<std::endl;
	
				while((sm_st.ReadMove(1))||(sm_st.ReadMove(2))||(sm_st.ReadMove(3))){
						usleep(10*1000);
					};
					
				// 打印测试
				std::cout<<"TEST！！"<<std::endl;

				Pos1_read = sm_st.ReadPos(1);
				if(Pos1_read!=-1){
					std::cout<<"pos1_current = "<<Pos1_read<<std::endl;
					usleep(10*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}
	
				Pos2_read = sm_st.ReadPos(2);
				if(Pos2_read!=-1){
					std::cout<<"pos2_current = "<<Pos2_read<<std::endl;
					usleep(10*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}
	
				Pos3_read = sm_st.ReadPos(3);
				if(Pos3_read!=-1){
					std::cout<<"pos3_current = "<<Pos3_read<<std::endl;
					usleep(10*1000);
				}else{
					std::cout<<"read pos err"<<std::endl;
					sleep(1);
				}
			}else{
				std::cout<<"错误：某一角度不在可行范围内！"<<std::endl;
			}	
		}
		else{
			std::cout<<"错误：超出软限位！"<<std::endl;
		}

}

void pf_control::pf_movel(double start_x, double start_y, double start_z, 
                  double end_x, double end_y, double end_z, 
                  double step, double t) {
        // 计算直线的总长度
        double distance = std::sqrt(
            std::pow(end_x - start_x, 2) +
            std::pow(end_y - start_y, 2) +
            std::pow(end_z - start_z, 2)
        );

        // 计算需要的总步数
        int num_steps = static_cast<int>(distance / step);
        if (num_steps == 0) num_steps = 1;  // 确保至少有一步

        // 每一步的增量
        double delta_x = (end_x - start_x) / num_steps;
        double delta_y = (end_y - start_y) / num_steps;
        double delta_z = (end_z - start_z) / num_steps;
  		// 每一步的运动时间
        double time_per_step = t / num_steps;

        // 循环执行每一步
        for (int i = 0; i <= num_steps; ++i) {
            double current_x = start_x + delta_x * i;
            double current_y = start_y + delta_y * i;
            double current_z = start_z + delta_z * i;
            // 调用 movep 函数执行单点运动

            pf_movep(current_x, current_y, current_z, time_per_step);
        }

    }

void pf_control::pf_readj(double &theta1, double &theta2, double &theta3){
	
	double Pos1_read,Pos2_read,Pos3_read;
	Pos1_read = sm_st.ReadPos(1);
	Pos2_read = sm_st.ReadPos(2);
	Pos3_read = sm_st.ReadPos(3);

	theta1 = (Pos1_read-900)*FT_STEP*5.0/10.5;
    theta2 = (Pos2_read-900)*FT_STEP*5.0/10.5;
    theta3 = (Pos3_read-900)*FT_STEP*5.0/10.5;

}

void pf_control::pf_readp(double &end_x, double &end_y, double &end_z){
	
	double theta1,theta2,theta3;
	pf_readj(theta1,theta2,theta3);
	theta1 = theta1/180.0*M_PI;
	theta2 = theta2/180.0*M_PI;
	theta3 = theta3/180.0*M_PI;
	pf_kin.PF_forward_kinematics(theta1,theta2,theta3,end_x,end_y,end_z);

}

void pf_control::pf_speedj(EV3d vel_theta){
	
	double theta1,theta2,theta3;
	pf_readj(theta1,theta2,theta3);

	// 判断是否接近角度边界
	if(theta1>=85||theta1<=5)
	{
		pf_speed_off();
		return;
	}

	if(theta2>=85||theta2<=5)
	{
		pf_speed_off();
		return;
	}
	if(theta3>=85||theta3<=5)
	{
		pf_speed_off();	
		return;
	}


	//打开恒速模式
	sm_st.WheelModeOn(1);
	sm_st.WheelModeOn(2);
	sm_st.WheelModeOn(3);

	//速度转换
	vel_theta[0] = vel_theta[0]*10.5/5.0/FT_STEP;
	vel_theta[1] = vel_theta[1]*10.5/5.0/FT_STEP;
	vel_theta[2] = vel_theta[2]*10.5/5.0/FT_STEP;

	//设置速度
	sm_st.WriteSpe(1, vel_theta[0], 50);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，旋转
	sm_st.WriteSpe(2, vel_theta[1], 50);//舵机(ID1)以加速度A=50(50*100步/秒^2)，停止旋转(V=0)
	sm_st.WriteSpe(3, vel_theta[2], 50);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，反向旋转


}

void pf_control::pf_speedp(EV3d vel_end){
	
	EV3d theta;
	EV3d vel_theta;
	double step = 0.02/180.0*M_PI;

	pf_readj(theta[0],theta[1],theta[2]);
	pf_kin.PF_inverse_vel(theta,vel_end,step,vel_theta);

	pf_speedj(vel_theta);
}

// 关闭速度控制模式
void pf_control::pf_speed_off(){
	//关闭恒速模式
	sm_st.WheelModeOff(1);
	sm_st.WheelModeOff(2);
	sm_st.WheelModeOff(3);
}