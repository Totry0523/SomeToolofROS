#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

#include <iostream>
#include <fstream>

void loadPointCloudFromBin(const std::string &file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::ifstream infile(file_path, std::ios::binary);
    if (!infile.is_open()) {
        std::cout << ("Could not open the .bin file.") << std::endl;
        return;
    }

    // Read the binary file
    pcl::PointXYZ point;
    while (infile.read(reinterpret_cast<char*>(&point), sizeof(pcl::PointXYZ))) {
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    std::cout << ("Loaded ") << cloud->points.size() << (" points.") << std::endl;
    cloud->height = 1; // Unorganized point cloud
    infile.close();
}

void visualizePointCloud(const std::string &file_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    loadPointCloudFromBin(file_path, cloud);

    // Create a visualizer
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    
    // Start visualization
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

int main(int argc, char **argv) {

    if (argc < 2) {
        std::cout << ("Usage: rosrun your_package_name point_cloud_visualizer <path_to_bin_file>") << std::endl;
        return -1;
    }

    std::string file_path = argv[1];
    visualizePointCloud(file_path);

    return 0;
}
