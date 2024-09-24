#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
    if (filenames.size() != 1)
    {
        std::cout << "Error: No pcd file given...\n";
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr InputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Average_Cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if ((pcl::io::loadPCDFile(argv[filenames[0]], *InputCloud) < 0))
    {
        std::cout << "Error: No pcd file given...\n";
        return -1;
    }
    
    std::cout << "Loaded point cloud with " << InputCloud->points.size() << " points." << std::endl;

    pcl::PointXYZ average_point(0,0,0);

    for(size_t i = 0; i < InputCloud->points.size(); i++)
    {
        average_point.x += InputCloud->points[i].x;
        average_point.y += InputCloud->points[i].y;
        average_point.z += InputCloud->points[i].z;
    }
    average_point.x /= InputCloud->points.size();
    average_point.y /= InputCloud->points.size();
    average_point.z /= InputCloud->points.size();
    Average_Cloud->push_back(average_point);

    std::cout << "Loaded point cloud with " << Average_Cloud->points.size() << " points." << std::endl;

    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> average_point_color_handler(InputCloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(InputCloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color_handler(InputCloud, 255, 0, 0);
    viewer.addPointCloud(InputCloud, source_cloud_color_handler, "original_cloud");
    viewer.addPointCloud(Average_Cloud, average_point_color_handler, "Average_Cloud");
    viewer.addCoordinateSystem(5.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "Average_Cloud");

    while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }

}