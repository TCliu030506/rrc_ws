#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#ifndef PI
#define PI 3.1415926535897932
#endif

class TF_fwd
{
public:
	TF_fwd();
	~TF_fwd();
  
  // 正运动学计算
  Eigen::Matrix4d dhfwardkmatics(double joint_val[6]);
  Eigen::Matrix4d tffwardkmatics(double (&joint)[6]);


private:

  Eigen::Matrix4d mat0_b, mat1_0, mat2_1, mat3_2, mat4_3, mat5_4, mat6_5;
  
  void ur5_param_load();
};
