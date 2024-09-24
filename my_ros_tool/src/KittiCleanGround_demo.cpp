#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <filesystem> // C++17

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

using namespace std;
typedef uint32_t LabelType;

struct labeled_point
{
    vector<LabelType> labels;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};


labeled_point* converter(const std::string &filename, const std::string &__frame) {
    const std::string filelabe_path = filename + "/labels/" + __frame +".label";
    const std::string filevelodyne_path = filename + "/velodyne/" + __frame + ".bin";

    std::vector<LabelType> __labels;
    pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::ifstream file_lable(filelabe_path, std::ios::binary);
    std::ifstream file_velodyne(filevelodyne_path, std::ios::binary);

    if (file_lable.is_open() && file_velodyne.is_open()) {
        LabelType label;
        pcl::PointXYZ point;

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
    std::cout << "pioint size: " << __cloud->size() << std::endl;

    labeled_point *return_type = new labeled_point;
    return_type->labels = __labels;
    return_type->cloud = __cloud;

    return return_type;
}


void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

std::string formatFrame(const std::string &frame){

    size_t zerotoadd = 6 - frame.size();
    return std::string(zerotoadd, '0') + frame;
}


int main(int agrc, char** argv) {

    const std::string path = argv[1];
    const std::string frame = argv[2];

    
    const std::string formatterframe = formatFrame(frame);
    labeled_point *data = converter(path,formatterframe);
    visualizePointCloud(data->cloud);
    delete data;

    return 0;
}