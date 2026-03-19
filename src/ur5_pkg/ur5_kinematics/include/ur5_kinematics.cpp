#include "ur5_kinematics.h"

// ur5_model_param
#define D1 0.08935245198864346
#define D4 0.1108629663680774
#define D5 0.09481752424615827
#define D6 0.08250304262136304
#define A2 0.4255142946511841
#define A3 0.3922769788

Eigen::Matrix4d URposition::dhfwardkmatics(double joint_val[6])
{
    Eigen::Matrix4d tm_matrix;
    double tm_solution[4][4];
	double c1 = 0, s1 = 0, c2 = 0, s2 = 0, c5 = 0, s5 = 0, c6 = 0, s6 = 0, c23 = 0, s23 = 0, c234 = 0, s234 = 0;
	c1 = cos(joint_val[0]);
	s1 = sin(joint_val[0]);
	c2 = cos(joint_val[1]);
	s2 = sin(joint_val[1]);
	c5 = cos(joint_val[4]);
	s5 = sin(joint_val[4]);
	c6 = cos(joint_val[5]);
	s6 = sin(joint_val[5]);
	c23 = cos(joint_val[1] + joint_val[2]);
	s23 = sin(joint_val[1] + joint_val[2]);
	c234 = cos(joint_val[1] + joint_val[2] + joint_val[3]);
	s234 = sin(joint_val[1] + joint_val[2] + joint_val[3]);

	tm_solution[0][0] = c1 * c234*c5*c6 + s1 * s5*c6 - c1 * s234*s6;
	tm_solution[0][1] = -c1 * c234*c5*s6 - s1 * s5*s6 - c1 * s234*c6;
	tm_solution[0][2] = -c1 * c234*s5 + s1 * c5;
	tm_solution[0][3] = -c1 * c234*s5*D6 + s1 * c5*D6 + c1 * s234*D5 - c1 * (A3*c23 + A2 * c2) + D4 * s1;
	tm_solution[1][0] = s1 * c234*c5*c6 - c1 * s5*c6 - s1 * s234*s6;
	tm_solution[1][1] = -s1 * c234*c5*s6 + c1 * s5*s6 - s1 * s234*c6;
	tm_solution[1][2] = -s1 * c234*s5 - c1 * c5;
	tm_solution[1][3] = -s1 * c234*s5*D6 - c1 * c5*D6 + s1 * s234*D5 - s1 * (A3*c23 + A2 * c2) - D4 * c1;
	tm_solution[2][0] = s234 * c5*c6 + c234 * s6;
	tm_solution[2][1] = -s234 * c5*s6 + c234 * c6;
	tm_solution[2][2] = -s234 * s5;
	tm_solution[2][3] = -s234 * s5*D6 - c234 * D5 - (A3*s23 + A2 * s2) + D1;
	tm_solution[3][0] = 0;
	tm_solution[3][1] = 0;
	tm_solution[3][2] = 0;
	tm_solution[3][3] = 1;

    tm_matrix << tm_solution[0][0],tm_solution[0][1],tm_solution[0][2],tm_solution[0][3],
                 tm_solution[1][0],tm_solution[1][1],tm_solution[1][2],tm_solution[1][3],
                 tm_solution[2][0],tm_solution[2][1],tm_solution[2][2],tm_solution[2][3],
                 tm_solution[3][0],tm_solution[3][1],tm_solution[3][2],tm_solution[3][3];



	return tm_matrix;
}



bool robot_pose_check(double (&joint)[6])
{
  Eigen::Vector2d link2, link3, link5;
  link2 << A2 * cos(PI + joint[1]), A2 * sin(PI + joint[1]);
  link3 << A3 * cos(PI + joint[1] + joint[2]), A3 * sin(PI + joint[1] + joint[2]);
  link5 << D5 * cos(PI + joint[1] + joint[2] + PI/2 + joint[3]), D5 * sin(PI + joint[1] + joint[2] + PI/2 + joint[3]);

  //eblow up
  Eigen::Vector2d link23, link23_unit, link23_normal;
  link23 = link2 + link3;
  link23_unit = link23 / link23.norm();

  link23_normal = link2 - (link2.dot(link23_unit)) * link23_unit;
  if(link23_normal.dot(Eigen::Vector2d::UnitY())<0) return false;
  
/*   //wrist down
  Eigen::Vector2d link235;
  link235 = link2 + link3 + link5;
  if(link235.dot(Eigen::Vector2d::UnitY())<0) return false; */

  return true;
}

/*******************************************************************************/
//class Dist
double Dist::distance[6] = {0,0,0,0,0,0};

Dist::Dist(/* args */)
{
  step = 1.0*PI/180;
}

Dist::~Dist(){}

void Dist::euler_trans(Eigen::Vector3d base, Eigen::Vector3d &euler)
{
  //goal(0) in (0,pi)
  if(fabs(euler(0)-base(0)) > (PI/2)){
    if(euler(0) < base(0)) euler(0) += PI;
    else euler(0) -= PI;

    double e0_ = PI - euler[1];
    euler[1]  = e0_;
    euler[2] -= PI;
  }
  shiftangle(euler(1),base(1));
  shiftangle(euler(2),base(2));
  return;
}

void Dist::trans(EIso3d pose_goal, EIso3d pose_now)
{
  Eigen::Vector3d r_goal = pose_goal.rotation().eulerAngles(0,1,2);
  Eigen::Vector3d r_now = pose_now.rotation().eulerAngles(0,1,2);
  //std::cout << "Goal" << r_goal.transpose() << std::endl;
  //std::cout << "Now" << r_now.transpose() << std::endl;

  euler_trans(r_now, r_goal);
  //std::cout << "Goal" << r_goal.transpose() << std::endl;

  for(unsigned long i=0;i<3;i++){
    p(i) = pose_goal.translation()(i) - pose_now.translation()(i);
    r(i) = r_goal(i) - r_now(i);
    distance[i]   = p(i);
    distance[3+i] = r(i);
  }
  return;
}

double Dist::lenth()
{
  pose.block<3,1>(0,0) = p * 1000;
  pose.block<3,1>(3,0) = r * 180.0/PI;
  return pose.norm();
}

EVec6d Dist::delta_pose()
{
  pose.block<3,1>(0,0) = p;
  pose.block<3,1>(3,0) = r;
  return pose;
}

/*******************************************************************************/
//URposition
URposition::URposition(){}

URposition::~URposition(){}

double URposition::jointbase[6] = {0,-PI/2,0,0,0,0};
EMat26d URposition::joint_limit = (EMat26d() << -PI*2, -PI, -PI*5/6, -PI*4/3, -PI*2, -PI*2,
                                                                                     PI*2,   0,  PI*5/6,    PI/3,  PI*2,  PI*2).finished();

void URposition::load_limit(EMat26d limit_in)
{
  joint_limit = limit_in;
}

void URposition::set_jointbase(double jointval_in[6])
{
	for(int i=0; i<6; i++) jointbase[i] = jointval_in[i];
}


bool URposition::inv_solve(EIso3d &mat, double (&joint)[6], int (&flag_in)[3], double joint6)
{
	// theta1
  double t1_ = pow(mat(0,3) - D6 * mat(0,2), 2) + pow(-mat(1,3) + D6 * mat(1,2), 2) - pow(D4, 2);
	if (t1_ < 0) {
		std::cout << "机械臂无法达到该范围,(x,y)需要远离基座标系原点！\n";
		return false;
	}
	joint[0] = atan2(D4, flag_in[0]*sqrt(t1_)) - atan2(-mat(1,3) + D6 * mat(1,2), mat(0,3) - D6 * mat(0,2));
	limitangle(joint[0]);
	double tc1 = 0;
	double ts1 = 0;
	tc1 = cos(joint[0]);
	ts1 = sin(joint[0]);

	// theta5
  double t5_ = ts1 * mat(0,2) - tc1 * mat(1,2);
	if (pow(t5_,2) > 1) {
		std::cout << "Rotation matrix is not qualified\n";
		return false;
	}
  joint[4] = atan2(flag_in[1]*sqrt(1.0 - pow(t5_,2)), t5_);
	limitangle(joint[4]);

	// theta6
	int flag6 = sign(sin(joint[4]));
  double theta234 = 0;
  if(flag6 == 0){
    joint[5] = joint6;
    limitangle(joint[5]);
    theta234 = atan2(mat(2,0), tc1*mat(0,0) + ts1 * mat(1,0)) - joint[5];
  }
  else{
    joint[5] = atan2(flag6*(-ts1 * mat(0,1) + tc1 * mat(1,1)), flag6*(ts1*mat(0,0) - tc1 * mat(1,0)));
	  limitangle(joint[5]);
    theta234 = atan2(-mat(2,2) * flag6, -(tc1*mat(0,2) + ts1 * mat(1,2))*flag6);
  }

	double ts5 = sin(joint[4]);
	double tc234 = cos(theta234);
	double ts234 = sin(theta234);

	// theta2
	double M = -tc1 * mat(0,3) - ts1 * mat(1,3) - tc234 * ts5*D6 + ts234 * D5;
	double N = D1 - mat(2,3) - ts234 * ts5*D6 - tc234 * D5;
	double mn = M * M + N * N + A2 * A2 - A3 * A3;

	double t2_ = 4.0 * A2*A2 * (M*M + N*N) - pow(mn, 2);
	if (t2_ < 0) {
		std::cout << "机械臂无法达到该范围，(x,y,z)不能超过半径750mm的球型空间!\n";
		return false;
	}
	joint[1] = atan2(mn, flag_in[2] * sqrt(t2_)) - atan2(M*A2, N*A2);
	limitangle(joint[1]);

	// theta3
	double tc2 = cos(joint[1]);
	double ts2 = sin(joint[1]);

	joint[2] = atan2(N - A2 * ts2, M - A2 * tc2) - joint[1];
	limitangle(joint[2]);

	// theta4
	joint[3] = theta234 - joint[1] - joint[2];
	limitangle(joint[3]);

	return true;
}



bool URposition::dhinvkinematics(EIso3d &mat, double (&joint)[6], double (&joint_base)[6])
{
	int flag[8][3] = { {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1}, {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1} };

  double solution[8][6];
	bool joint_val_check[8];
	bool has_solution = false;

	for(int i=0; i<8; i++){
		if(inv_solve(mat, solution[i], flag[i], joint_base[5])){
      has_solution = true;
      shiftangle(solution[i],joint_base);
			joint_val_check[i] = check_joint(joint_limit, solution[i]);
      //printf solution
      if(false){
        std::cout << "fail one solution:" << std::endl;
        std::cout << solution[i][0]*180/PI << ", " << solution[i][1]*180/PI << ", " << solution[i][2]*180/PI << ", "  
                  << solution[i][3]*180/PI << ", " << solution[i][4]*180/PI << ", " << solution[i][5]*180/PI << std::endl;
      }
		}
		else{
			joint_val_check[i] = false;
		}
	}

	if(!has_solution){
		printf("Solutions all failed\n");
		return false;
	}

  //check
  for(int i=0; i<8; i++) if(joint_val_check[i]) if(!robot_pose_check(solution[i])) {joint_val_check[i] = false;
  if(false){
    std::cout << "fail one solution plus:" << std::endl;
    std::cout << solution[i][0]*180/PI << ", " << solution[i][1]*180/PI << ", " << solution[i][2]*180/PI << ", "  
              << solution[i][3]*180/PI << ", " << solution[i][4]*180/PI << ", " << solution[i][5]*180/PI << std::endl;
    }
  }
      

	// select solution
	int choice = 8;
  double distance = 12*PI;
  for(int i=0; i<8; i++){
    if(joint_val_check[i]){ 
      double dis_sum = 0;
      for(unsigned long j=0;j<6;j++) dis_sum += fabs(solution[i][j] - joint_base[j]);
      if(distance > dis_sum){
        distance = dis_sum;
        choice = i;
      }
    }
  }

	if(choice>=0 && choice<=7){
		for(int i=0; i<6; i++) joint[i] = solution[choice][i];
		//printf("Adopted Inverse kinematics solution: [%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f]\n\n",joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
		return true;
	}else{
		//printf("failed\n\n");
		return false;
	}
}

void URposition::select_mat(EIso3d &mat, int type)
{
	Eigen::Matrix3d rotate = Eigen::Matrix3d::Identity();
    Eigen::Vector3d point;
    Eigen::AngleAxisd ang;
	Eigen::Vector3d eul;
	switch (type)
    {
	//角轴表示
    case 1:
    {
      std::cout << "角轴表示输入格式:" << std::endl;
      std::cout << "角度(°) 轴X(m) 轴Y(m) 轴Z(m) 点X(m) 点Y(m) 点Z(m)" << std::endl;
      std::cin >> ang.angle() >> ang.axis()[0] >> ang.axis()[1] >> ang.axis()[2] >> point(0) >> point(1) >> point(2); 
      ang.angle() = ang.angle()/180*PI;

      rotate = ang.matrix();

	  // 设置旋转矩阵
	  mat.linear() = rotate;
	  // 设置平移向量
	  mat.translation() = Eigen::Vector3d(point[0], point[1], point[2]);

      break;
    }
	//欧拉角表示
    case 2:
    {
      std::cout << "欧拉角输入格式：" << std::endl;
      std::cout << "角度1(°) 角度2(°) 角度3(°) 点X(m) 点Y(m) 点Z(m)" << std::endl;
      std::cin >> eul(0) >> eul(1) >> eul(2) >> point(0) >> point(1) >> point(2); 
      eul = eul/180*PI;
      Eigen::AngleAxisd eul_x(Eigen::AngleAxisd(eul(0),Eigen::Vector3d::UnitX()));
      Eigen::AngleAxisd eul_y(Eigen::AngleAxisd(eul(1),Eigen::Vector3d::UnitY()));
      Eigen::AngleAxisd eul_z(Eigen::AngleAxisd(eul(2),Eigen::Vector3d::UnitZ()));
      ang = eul_x * eul_y * eul_z;
      Eigen::Vector3d eul_2;

      rotate = ang.matrix();
	  // 设置旋转矩阵
	  mat.linear() = rotate;
	  // 设置平移向量
	  mat.translation() = Eigen::Vector3d(point[0], point[1], point[2]);

      break;
    }
	//关节表示
	//-10 -80 -100 -150 -100 -100
	//-10 -80 -100 -10 90 90
    case 3:
    {
      double joint6[6];
      std::cout << "关节角度输入格式：" << std::endl;
      std::cout << "关节1(°) 关节2(°) 关节3(°) 关节4(°) 关节5(°) 关节6(°)" << std::endl;
      std::cin >> joint6[0] >> joint6[1] >> joint6[2] >> joint6[3] >> joint6[4] >> joint6[5]; 
      for(unsigned long i=0;i<6;i++) joint6[i] = joint6[i]/180*PI;

	  Eigen::Matrix4d tm_matrix = dhfwardkmatics(joint6);
	  mat = tm_matrix;

      break;
    }

	//四元数
	//0.2 0.2 0.3 0.1 0.1 0.1 0.1 
	//0.4 0.1 0.3 0.2 0.2 0.2 0.1
	case 4:
	{
		// 初始化一个数组用于存储输入的位置向量
		double position[3];
		// 初始化一个数组用于存储输入的四元数
		double quaternion[4];
		// 提示用户输入格式
		std::cout << "输入格式为:" << std::endl;
		std::cout << "位置X(m) 位置Y(m) 位置Z(m) 四元数w 四元数x 四元数y 四元数z" << std::endl;
		// 接收用户输入的位置向量和四元数
		std::cin >> position[0] >> position[1] >> position[2] 
				>> quaternion[0] >> quaternion[1] >> quaternion[2] >> quaternion[3];
		
		
		// 创建四元数
		Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
		// 正规化四元数，以防输入未正规化
		q.normalize();
		// 将四元数转换为旋转矩阵
		Eigen::Matrix3d m_rot = q.toRotationMatrix();

		// 设置旋转矩阵
		mat.linear() = m_rot;
		// 设置平移向量
		mat.translation() = Eigen::Vector3d(position[0], position[1], position[2]);

		break;
	}

	}

}