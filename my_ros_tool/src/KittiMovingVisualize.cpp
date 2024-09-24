#include <iostream>
#include <fstream>
#include <cstdio>
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_static;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_moving;
};


labeled_point* converter(const std::string &filename, const std::string &__frame) {

    const std::string filelabe_path = filename + "/labels/" + __frame +".label";
    const std::string filevelodyne_path = filename + "/velodyne/" + __frame + ".bin";

    std::vector<LabelType> __labels;
    pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_static(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_moving(new pcl::PointCloud<pcl::PointXYZ>);

    std::ifstream file_lable(filelabe_path, std::ios::binary);
    std::ifstream file_velodyne(filevelodyne_path, std::ios::binary);

    if (file_lable.is_open() && file_velodyne.is_open()) {
        LabelType label;
        pcl::PointXYZ point;

        while (file_lable.read(reinterpret_cast<char*>(&label), sizeof(LabelType)) &&
               file_velodyne.read(reinterpret_cast<char*>(&point), sizeof(pcl::PointXYZ))) {
            if (label < 200) {
                __labels.push_back(label);
                __cloud_static->push_back(point);
            }else{
                __cloud_moving->push_back(point);
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
    std::cout << "static pioint size: " << __cloud_static->size() << std::endl;
    std::cout << "moving pioint size: " << __cloud_moving->size() << std::endl;

    labeled_point *return_type = new labeled_point;
    return_type->labels = __labels;
    return_type->cloud_static = __cloud_static;
    return_type->cloud_moving = __cloud_moving;

    return return_type;
}

std::string formatFrame(const std::string &frame){

    size_t zerotoadd = 6 - frame.size();
    return std::string(zerotoadd, '0') + frame;
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

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_static, const pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_moving){

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> static_cloud_color_handler(__cloud_static, 0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> moving_cloud_color_handler(__cloud_moving, 255, 0, 0);
    viewer.setBackgroundColor(255, 255, 255);

    viewer.addPointCloud<pcl::PointXYZ>(__cloud_static, static_cloud_color_handler, "static cloud");
    viewer.addPointCloud<pcl::PointXYZ>(__cloud_moving, moving_cloud_color_handler, "moving cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "static cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "moving cloud");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

int main(int agrc, char** argv) {

    const std::string path = argv[1];
    const std::string frame = argv[2];

    const std::string formatterframe = formatFrame(frame);
    // const std::string formatterframe = frame;
    labeled_point *data = converter(path,formatterframe);
    // visualizePointCloud(data->cloud);
    visualizePointCloud(data->cloud_static, data->cloud_moving);
    delete data;

    return 0;
}