#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

#define FILE_PATH "/zzrobot_ws/src/ur5lzh_pkg/pointcloudslam_cpp/data/"
#define FILE_NAME "areapicked"

void polyfit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, Eigen::VectorXd& coefficients)
{
    size_t n = cloud_in->points.size();
    Eigen::MatrixXd A(n, 21);
    Eigen::VectorXd B(n);
    double x,y,z;
    for(size_t i=0;i<n;i++)
    {
        x = cloud_in->points[i].x;
        y = cloud_in->points[i].y;
        z = cloud_in->points[i].z;
        
        A(i, 0) = 1;
        A(i, 1) = x;
        A(i, 2) = y;
        A(i, 3) = x * x;
        A(i, 4) = x * y;
        A(i, 5) = y * y;
        A(i, 6) = x * x * x;
        A(i, 7) = x * x * y;
        A(i, 8) = x * y * y;
        A(i, 9) = y * y * y;
        A(i, 10) = x * x * x * x;
        A(i, 11) = x * x * x * y;
        A(i, 12) = x * x * y * y;
        A(i, 13) = x * y * y * y;
        A(i, 14) = y * y * y * y;
        A(i, 15) = x * x * x * x * x;
        A(i, 16) = x * x * x * x * y;
        A(i, 17) = x * x * x * y * y;
        A(i, 18) = x * x * y * y * y;
        A(i, 19) = x * y * y * y * y;
        A(i, 20) = y * y * y * y * y;

        B(i) = z;
    }
    coefficients = A.colPivHouseholderQr().solve(B);
}
double differential_y(double x,double y,Eigen::VectorXd& coefficients)
{
    double sum = 0;
    sum += coefficients(0) * 0;
    sum += coefficients(1) * 0;
    sum += coefficients(2) * 1.0;
    sum += coefficients(3) * 0;
    sum += coefficients(4) * x;
    sum += coefficients(5) * 2*y;
    sum += coefficients(6) * 0;
    sum += coefficients(7) * x*x;
    sum += coefficients(8) * 2*y*x;
    sum += coefficients(9) * 3*y*y;
    sum += coefficients(10) * 0;
    sum += coefficients(11) * x*x*x;
    sum += coefficients(12) * 2*x*x*y;
    sum += coefficients(13) * 3*y*y*x;
    sum += coefficients(14) * 4*y*y*y;
    sum += coefficients(15) * 0;
    sum += coefficients(16) * x*x*x*x;
    sum += coefficients(17) * 2*x*x*x*y;
    sum += coefficients(18) * 3*x*x*y*y;
    sum += coefficients(19) * 4*x*y*y*y;
    sum += coefficients(20) * 5*y*y*y*y;
    return sum;
}

double differential_x(double x,double y,Eigen::VectorXd& coefficients)
{
    double sum = 0;
    sum += coefficients(0) * 0;
    sum += coefficients(1) * 1.0;
    sum += coefficients(2) * 0;
    sum += coefficients(3) * 2*x;
    sum += coefficients(4) * y;
    sum += coefficients(5) * 0;
    sum += coefficients(6) * 3*x*x;
    sum += coefficients(7) * 2*x*y;
    sum += coefficients(8) * y*y;
    sum += coefficients(9) * 0;
    sum += coefficients(10) * 4*x*x*x;
    sum += coefficients(11) * 3*x*x*y;
    sum += coefficients(12) * 2*x*y*y;
    sum += coefficients(13) * y*y*y;
    sum += coefficients(14) * 0;
    sum += coefficients(15) * 5*x*x*x*x;
    sum += coefficients(16) * 4*x*x*x*y;
    sum += coefficients(17) * 3*x*x*y*y;
    sum += coefficients(18) * 2*x*y*y*y;
    sum += coefficients(19) * y*y*y*y;
    sum += coefficients(20) * 0;
    return sum;
}

int main(int argc, char** argv)
{
	std::string file_name;
	std::cin >> file_name;
	std::cin.ignore();
	std::string pcd_path = std::string(std::getenv("HOME")) + FILE_PATH + file_name +".pcd";
	// 从PCD文件中读取点云数据
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read(pcd_path, *cloud);
    size_t n = cloud->points.size();
	std::cout << "PointCloud has: " << n << " data points." << std::endl;

    Eigen::VectorXd coefficients(21);

    polyfit(cloud, coefficients);
    std::cout<<"coefficients contents:\n";
    std::cout<<coefficients<<std::endl;
    //max min
    double x_min = cloud->points[0].x;
    double x_max = cloud->points[0].x;
    double y_min = cloud->points[0].y;
    double y_max = cloud->points[0].y;
    for (size_t i=0;i<n;i++)
    {
        if(cloud->points[i].x<x_min) x_min = cloud->points[i].x;
        if(cloud->points[i].x>x_max) x_max = cloud->points[i].x;
        if(cloud->points[i].y<y_min) y_min = cloud->points[i].y;
        if(cloud->points[i].y>y_max) y_max = cloud->points[i].y;
    }

 
	return (0);
}