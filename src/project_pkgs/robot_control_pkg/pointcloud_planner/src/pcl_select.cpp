

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <vtkAreaPicker.h>
#include <string>
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
typedef pcl::PointXYZ PointType;

#define FILE_PATH "/zzrobot_ws/src/ur5lzh_pkg/pointcloudslam_cpp/data/"
 
void areapickingcallback(const pcl::visualization::AreaPickingEvent &event,void *userdata)
{
    std::string output_path;
    output_path = std::string(std::getenv("HOME")) + FILE_PATH + "areapicked.pcd";
    pcl::PointCloud<PointType>::Ptr  secloud(new pcl::PointCloud<PointType>());
    std::cout<<"Into here"<<std::endl;
    std::vector<int> indices;
    event.getPointsIndices(indices);
    pcl::IndicesPtr ind_plane=std::make_shared<std::vector<int>>(indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);                       //导入点云数据
    extract.setIndices(ind_plane);                      //设置点云索引
    extract.setNegative(false);                  //设置为false，选择索引指向的点导出
    extract.filter(*secloud);                         //输出所选点云
    std::cout<<"Nums selected\t"<<secloud->points.size()<<std::endl;
    pcl::io::savePCDFile(output_path,*secloud);
}
int main() 
{
	std::string file_name;
    std::cin >> file_name;
    std::cin.ignore();
    std::string pcd_path = std::string(std::getenv("HOME")) + FILE_PATH + file_name + ".pcd";

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Carviewer"));
    pcl::io::loadPCDFile(pcd_path,*cloud);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud(cloud,"car");
    viewer->registerAreaPickingCallback(areapickingcallback);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
