#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>
#include <iostream>

// #define FILE_PATH "/zzrobot_ws/src/ur5lzh_pkg/pointcloudslam_cpp/data/"
#define FILE_PATH "/zzrobot_ws/src/ur5lzh_pkg/pointcloud_process/data/back_model/"

int main(int argc, char** argv)
{
	std::cout<<"Input the file name.\n";
	std::string file_name;
	std::cin >> file_name;
	std::cin.ignore();
	// 支持三种输入情况：
	// 1) 以 '/' 开头 -> 视为绝对路径
	// 2) 包含 '/' 或以 '.' 开头 -> 视为相对路径（相对于当前工作目录）
	// 3) 纯文件名 -> 使用默认宏 FILE_PATH 下的文件（保持向后兼容）
	std::string pcd_path;
	if (!file_name.empty() && file_name[0] == '/')
	{
		pcd_path = file_name;
	}
	else if (file_name.find('/') != std::string::npos || (!file_name.empty() && file_name[0] == '.'))
	{
		std::filesystem::path p = std::filesystem::current_path() / file_name;
		pcd_path = p.string();
	}
	else
	{
		const char* home_c = std::getenv("HOME");
		if (!home_c)
		{
			std::cerr << "Environment variable HOME is not set.\n";
			return -1;
		}
		pcd_path = std::string(home_c) + FILE_PATH + file_name + ".pcd";
	}

	// 如果输入没有扩展名且路径没有以 .pcd 结尾，为便捷起见自动追加 .pcd
	if (pcd_path.size() < 4 || pcd_path.substr(pcd_path.size()-4) != ".pcd")
	{
		pcd_path += ".pcd";
	}

	// 打印最终使用的路径，便于调试
	std::cerr << "Using PCD path: " << pcd_path << "\n";
	// 从PCD文件中读取点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 检查文件是否存在并尝试加载，先不创建可视化器以避免在无 DISPLAY 时触发 VTK 崩溃
	try {
		if (!std::filesystem::exists(pcd_path))
		{
			std::cerr << "PCD file not found: " << pcd_path << "\n";
			return -1;
		}
	} catch (const std::exception &e) {
		std::cerr << "Filesystem check failed: " << e.what() << "\n";
		return -1;
	}

	int ret = pcl::io::loadPCDFile(pcd_path, *cloud);
	if (ret == -1 || cloud->empty())
	{
		std::cerr << "Failed to load PCD file: " << pcd_path << " (return=" << ret << ")\n";
		return -1;
	}

	// 如果没有 X DISPLAY，跳过可视化（避免 VTK/PCLVisualizer 在 headless 环境中崩溃）
	if (!std::getenv("DISPLAY"))
	{
		std::cerr << "DISPLAY not set: skipping visualization. Loaded " << cloud->size() << " points.\n";
		return 0;
	}

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Pointcloud"));
	viewer->setBackgroundColor(1.0,1.0,1.0);
	viewer->addPointCloud(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,0.0,0.0,"cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	return (0);
}