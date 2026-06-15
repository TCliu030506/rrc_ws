

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <vtkAreaPicker.h>
#include <string>
#include <cstdlib>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
typedef pcl::PointXYZ PointType;

std::string output_path;
bool is_areapicked = false;
 
void areapickingcallback(const pcl::visualization::AreaPickingEvent &event,void *userdata)
{
    (void)userdata;
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
    is_areapicked = true;
    std::exit(0);
}
int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <output.pcd>" << std::endl;
        return 1;
    }

    const std::string pcd_path = argv[1];
    output_path = argv[2];
    std::cout << "Loading point cloud: " << pcd_path << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Carviewer"));
    if (pcl::io::loadPCDFile(pcd_path,*cloud) < 0) {
        std::cerr << "Failed to load point cloud: " << pcd_path << std::endl;
        return 1;
    }
    std::cout << "Loaded " << cloud->points.size() << " points. Select ROI in the viewer window." << std::endl;
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud(cloud,"car");
    viewer->registerAreaPickingCallback(areapickingcallback);
    viewer->spin();
    std::cerr << "ROI selection window closed before selecting points." << std::endl;
    return 1;
}
