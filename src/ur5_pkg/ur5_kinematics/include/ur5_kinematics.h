#ifndef UR5_KINEMATICS_H
#define UR5_KINEMATICS_H


#ifndef PI
#define PI 3.1415926535897932
#endif

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <string>

#include <iostream>
#include <cmath> // 用于M_PI常量

typedef Eigen::Matrix<double,6,6> EMat6d;
typedef Eigen::Matrix3d           EMat3d;
typedef Eigen::Matrix<double,3,6> EMat36d;
typedef Eigen::Matrix<double,2,6> EMat26d;
typedef Eigen::Isometry3d         EIso3d;
typedef Eigen::Matrix<double,6,1> EVec6d;






// class to calculate the distance of pose(mm/du)
class Dist
{
private:
  EVec6d pose;              //data used for returning
  Eigen::Vector3d p, r;

  double step;

  //limit rx to (-pi/2,pi/2)
  void euler_trans(Eigen::Vector3d base, Eigen::Vector3d &euler);

public:
  Dist(/* args */);
  ~Dist();

  static double distance[6];    //data used for returning

  //trans from pose_now to pose_goal
  void trans(EIso3d pose_goal, EIso3d pose_now);

  //calculate the length of the matrix<6,1> base on mm & du
  double lenth();

  //return delta_pose
  EVec6d delta_pose();

};

// class to handle forward and inverse kinematics
class URposition
{
public:
	URposition();
	~URposition();

  static double jointbase[6];					              //give the reference of the inverse kinematics

  static Eigen::Matrix<double,2,6> joint_limit;      //joint limit, set using set_limit function 

  //set joint limit
  void load_limit(Eigen::Matrix<double,2,6> limit_in);
  
  // update current position manually to jointbase
  void set_jointbase(double jointval_in[6]);
  
  // 正运动学计算
  Eigen::Matrix4d dhfwardkmatics(double joint_val[6]);

  //calculate inverse kinematics
  bool dhinvkinematics(EIso3d &mat, double (&joint)[6], double (&joint_base)[6]);

  // 位姿表示的多种输入形式最后转换为旋转矩阵
  void select_mat(EIso3d &mat, int type);

private:

  //give inverse kinematics solution follow the flag
	bool inv_solve(EIso3d &mat, double (&joint)[6], int (&flag_in)[3], double joint6);

  
};


//return -1,0,1
template<typename T> int sign(T val)
{
  if(val > 0.0001) return 1;
  else if(val < -0.0001) return -1;
  else return 0;
};





//limit the joint value in (-pi,pi]	
inline void limitangle(double &theta)
{
	while(theta > PI) theta -= 2.0*PI;
	while(theta <= (-PI+0.0001)) theta += 2.0*PI;
};

inline void limitangle(double (&theta)[6])
{
  for(unsigned int i=0;i<6;i++) limitangle(theta[i]);
};

//shift the joint value base on joint	
inline void shiftangle(double &theta, double base)
{
	while(theta > (base + PI)) theta -= 2.0*PI;
	while(theta < (base - PI + 0.0001)) theta += 2.0*PI;
};

inline void shiftangle(double (&theta)[6], double (&base)[6])
{
  for(unsigned int i=0;i<6;i++) shiftangle(theta[i], base[i]);
};

//check if joint_val is in limit, in limit: true
inline bool check_joint(EMat26d limit, unsigned long index, double joint)
{
  if(joint < limit(0,index)) return false;
  if(joint > limit(1,index)) return false;
	return true;
};

inline bool check_joint(EMat26d limit, double (&joint)[6])
{
	for (unsigned long i = 0; i < 6; i++) {
    double val = joint[i];
    while(val <= limit(0,i)) val += PI * 2.0;
    while(val >= limit(1,i)) val -= PI * 2.0;
    if(val <= limit(0,i)) {
      //std::cout << "\njoint fail" << joint[i] << ", limit1:" << i << ":" << limit(1,i) << std::endl;
      return false;
    }
    else joint[i] = val;
	}
	return true;
};

//check robot state
bool robot_pose_check(double (&joint)[6]);







#endif // UR5_KINEMATICS_H