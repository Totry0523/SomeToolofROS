# SomeToolofROS

## my_pcl_tool
viewbin.cpp 查看一个.bin格式的点云
viewpcd.cpp 查看一个.pcd格式的点云

## my_ros_tool
KittiCleanGround_demo.cpp 一个去除KITTI数据集地面点（label=40）并可视化的demo

KittiCleanGroundAndSave.cpp 去除所有文件的地面点并保存到另一个文件夹
'rosrun my_ros_tool KittiCleanGroundAndSave </path/to/input> </path/to/output>'
KittiMovingVisualize.cpp 可视化点云的移动物体点
'rosrun my_ros_tool KittiCleanGroundAndSave </path/to/tragetdir>'
