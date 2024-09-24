#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <filesystem> // C++17

#include <boost/filesystem.hpp>  // 使用 Boost 文件系统库
#include <boost/format.hpp>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

using namespace std;
typedef uint32_t LabelType;


void converter(const std::string &filename, const std::string &output_path) {

    // const std::string filelabe_path = filename + "/labels/000000.label";
    // const std::string filevelodyne_path = filename + "/velodyne/000000.bin";
    // const std::string output_label_path = output_path + "/labels/000000.label";
    // const std::string output_velodyne_path = output_path + "/velodyne/000000.bin";
    LabelType label;
    pcl::PointXYZ point;

    for(int num_files = 0; ; num_files++){
        std::string filelabe_path = (boost::format("%s/labels/%06d.label") % filename % num_files).str();
        std::string filevelodyne_path = (boost::format("%s/velodyne/%06d.bin") % filename % num_files).str();


        std::string output_label_path = (boost::format("%s/labels/%06d.label") % output_path % num_files).str();
        std::string output_velodyne_path = (boost::format("%s/velodyne/%06d.bin") % output_path % num_files).str();

        if (!boost::filesystem::exists(filelabe_path) || !boost::filesystem::exists(filevelodyne_path)) {
            break; // 文件不存在时退出循环
        }

        cout << "\033[1;34m 正在处理第: " << num_files  << "帧\033[0m"<< endl;

        std::vector<LabelType> __labels;
        pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud(new pcl::PointCloud<pcl::PointXYZ>);

        std::ifstream file_lable(filelabe_path, std::ios::binary);
        std::ifstream file_velodyne(filevelodyne_path, std::ios::binary);

        if (file_lable.is_open() && file_velodyne.is_open()) {

            while (file_lable.read(reinterpret_cast<char*>(&label), sizeof(LabelType)) &&
                file_velodyne.read(reinterpret_cast<char*>(&point), sizeof(pcl::PointXYZ))) {
                if (label != 40) {
                    __labels.push_back(label);
                    __cloud->push_back(point);
                }
                // 否则跳过地面点
            }

            if (file_lable.bad()) {
                std::cerr << "读取标签文件时发生错误。" << std::endl;
            }

            if (file_velodyne.bad()) {
                std::cerr << "读取点云文件时发生错误。" << std::endl;
            }

            file_lable.close();
            file_velodyne.close();
        } else {
            std::cerr << "无法打开文件 " << filelabe_path << " 和/或 " << filevelodyne_path << std::endl;
        }

        // 将处理后的标签保存为新文件
        std::ofstream output_label(output_label_path, std::ios::binary);
        if (output_label.is_open()) {
            for (const auto& lbl : __labels) {
                output_label.write(reinterpret_cast<const char*>(&lbl), sizeof(LabelType));
            }
            output_label.close();
        } else {
            std::cerr << "无法写入标签文件: " << output_label_path << std::endl;
        }

        // 将处理后的点云保存为新文件
        std::ofstream output_velodyne(output_velodyne_path, std::ios::binary);
        if (output_velodyne.is_open()) {
            for (const auto& point : __cloud->points) {
                output_velodyne.write(reinterpret_cast<const char*>(&point), sizeof(pcl::PointXYZ));
            }
            output_velodyne.close();
        } else {
            std::cerr << "无法写入点云文件: " << output_velodyne_path << std::endl;
        }

        cout << "\033[1;35m 第: " << num_files  << "帧处理完成\033[0m"<< endl;

        }

}

int main(int argc, char** argv) {

    if (argc < 3) {
        std::cerr << "help: " << argv[0] << " <input文件夹> <output文件夹>" << std::endl;
        return -1;
    }
    const std::string path_in = argv[1];
    const std::string path_out = argv[2];

    converter(path_in,path_out);

    return 0;
}