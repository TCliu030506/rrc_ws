#include <../include/tf_fwd.h>

TF_fwd::TF_fwd(){
	ur5_param_load();
}
TF_fwd::~TF_fwd(){}

Eigen::Matrix4d eulrpy2mat(double (&pose_rpy)[6])
{
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	for(int i=0;i<3;i++) mat(i,3) = pose_rpy[i];
	Eigen::AngleAxisd roll (pose_rpy[3],Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitch(pose_rpy[4],Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yaw  (pose_rpy[5],Eigen::Vector3d::UnitZ());
	mat.block<3, 3>(0, 0) = yaw.matrix()*pitch.matrix()*roll.matrix();
	return mat;
};
Eigen::Matrix4d jointrotation(double joint, Eigen::Matrix4d mat_in)
{
	Eigen::AngleAxisd rotate(joint,Eigen::Vector3d::UnitZ());
	Eigen::Matrix4d rotm = Eigen::Matrix4d::Identity();
	rotm.block<3, 3>(0, 0) = rotate.matrix();
	Eigen::Matrix4d mat_out = rotm*mat_in;
	return mat_out;
}


void TF_fwd::ur5_param_load()
{
	// URDF:              ([t1r1]*r(joint1))  *  ([t2r2]*r(joint2)) *  ([t3r3]*r(joint3)) *   ([t4r4]*r(joint4)) *  ([t5r5]*r(joint5)) *  ([t6r6]*r(joint6))
	// ->         [t1r1] *  r(joint1)*[t2r2]  *    r(joint2)*[t3r3] *    r(joint3)*[t4r4] *     r(joint4)*[t5r5] *    r(joint5)*[t6r6] *   r(joint6)
	// ->  [T1*J0Z_inv]  *    R1*[J0Z*T2]     *      R2*T3          *   R3*[T4*J3Z_inv]   *  R4*[J3Z*T5*J4Z_inv] * R5*[J4Z*T6*J5Z_inv] *   R6*J5Z
	// DH:                (R(joint1)*T1*R1)   *  (R(joint2)*T2*R2)  *  (R(joint3)*T3*R3)  *   (R(joint4)*T4*R4)  *  (R(joint5)*T5*R5)  *  (R(joint6)*T6*R6)

	//trans0_b: [T1*J0Z_inv]
	//Transmat_0_b: Pose_RPY(0,0,0,0,0,0)
	mat0_b = Eigen::Matrix4d::Identity();
	double pose_rpy_shoulder[6] = {0,0,0.08935245198864346,0,0,2.704901158773852e-05};
	//read_yaml("shoulder",pose_rpy);
	Eigen::Matrix4d tool;
	tool = eulrpy2mat(pose_rpy_shoulder);

	Eigen::Matrix4d t0_z = Eigen::Matrix4d::Identity();
	t0_z(2,3) = pose_rpy_shoulder[2];

	mat0_b = tool * t0_z.matrix().inverse();

	//trans1_0: R1*[J0Z*T2]
	//Transmat_1_0: Pose_RPY(0,0,D1,pi/2,0,theta1)
	mat1_0 = Eigen::Matrix4d::Identity();
	double pose_rpy_upperarm[6] = {0.0001185558467403625,0,0,1.568912757202799,0,-3.921502200784356e-05};
	//read_yaml("upper_arm",pose_rpy);
	tool = eulrpy2mat(pose_rpy_upperarm);

	mat1_0 = t0_z * tool;

	//trans2_1: R2*T3
	//Transmat_2_1: Pose_RPY(-A2*cos(theta2),-A2*sin(theta2),0,0,0,theta2)
	mat2_1 = Eigen::Matrix4d::Identity();
	double pose_rpy_forearm[6] = {-0.4255142946511841,0,0,3.14024628319171,3.139484860886727,3.141579462304971};
	//read_yaml("forearm",pose_rpy);
	tool = eulrpy2mat(pose_rpy_forearm);
	mat2_1 = tool;

	//trans3_2: R3*[T4*J3Z_inv]
	//Transmat_3_2: Pose_RPY(-A3*cos(theta3),-A3*sin(theta3),0,0,0,theta3)
	mat3_2 = Eigen::Matrix4d::Identity();
	double pose_rpy_wrist1[6] = {-0.3922767533843762,0.0004204946371233512,0.1108629663680774,3.13779974930022,3.140506260447464,3.141535657644125};
	//read_yaml("wrist_1",pose_rpy);
	tool = eulrpy2mat(pose_rpy_wrist1);

	Eigen::Matrix4d t3_z = Eigen::Matrix4d::Identity();
	t3_z(2,3) = pose_rpy_wrist1[2];

	mat3_2 = tool * t3_z.matrix().inverse();

	//trans4_3: R4*[J3Z*T5*J4Z_inv]
	//Transmat_4_3: Pose_RPY(0,0,D4,pi/2,0,theta4)
	mat4_3 = Eigen::Matrix4d::Identity();
	double pose_rpy_wrist2[6] = {8.033676097425756e-05,-0.09481752424615827,7.771429447338641e-05,1.569976707450381,0,7.218452370066675e-05};
	//read_yaml("wrist_2",pose_rpy);
	tool = eulrpy2mat(pose_rpy_wrist2);

	Eigen::Matrix4d t4_z = Eigen::Matrix4d::Identity();
	t4_z(2,3) = -pose_rpy_wrist2[1];  

	mat4_3 = t3_z * tool * t4_z.matrix().inverse();

	//trans5_4: R5*[J4Z*T6*J5Z_inv]
	//Transmat_5_4: Pose_RPY(0,0,D5,-pi/2,0,theta5)
	mat5_4 = Eigen::Matrix4d::Identity();
	//read_yaml("wrist_3",pose_rpy);
	
	double pose_rpy_wrist3[6] = {2.265173098375042e-05,0.08250304262136304,2.735526838103436e-05,1.571127893565156,3.141592653589793,-3.141537130532523};
	tool = eulrpy2mat(pose_rpy_wrist3);
	Eigen::Matrix4d t5_z = Eigen::Matrix4d::Identity();
	t5_z(2,3) = pose_rpy_wrist3[1];

	mat5_4 = t4_z * tool * t5_z.matrix().inverse();

	//trans6_5: R6*J5Z
	//Transmat_6_5: Pose_RPY(0,0,D6,0,0,theta6)
	mat6_5 = Eigen::Matrix4d::Identity();
	mat6_5 = t5_z;
}

Eigen::Matrix4d TF_fwd::tffwardkmatics(double (&joint)[6])
{
  	Eigen::Matrix4d mat;

	mat = mat0_b*jointrotation(joint[0],mat1_0)*jointrotation(joint[1],mat2_1)*jointrotation(joint[2],mat3_2)*jointrotation(joint[3],mat4_3)*jointrotation(joint[4],mat5_4)*jointrotation(joint[5],mat6_5);
	return mat;
}