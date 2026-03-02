项目机械臂主要代码位于      小车整体运行代码\src\E2E-Sample\ros2_robot_arm\ros2_ws\build/dofbot_garbage_yolov5/build/lib/dofbot_garbage_yolov5文件夹下
项目完整结构如下：
dofbot_garbage_yolov5
|_dofbot_info
|_dofbot_moveit
|_robot_arm_bringup
|_robot_arm_color_stacking
|_dofbot_garbage_yolov5

其中dofbot_garbage_yolov5以及dofbot_moveit与dofbot_info为本项目的主要文件夹
其中主要项目dofbot_garbage_yolov5的文件结构如下：
dofbot_garbage_yolov5
|_utils
   |_det_utils.py
   |_dofbot_config.py
   |_garbage_grab.py
   |_garbage_identify.py
   |_npu_utils.py
|_main.py
|_main_only_find.py
|_test.py

其中utils文件夹下主要存放了npu推理的基本函数实现、Yolov8推理的实现代码以及机械臂和机械手执行动作的代码以及大模型推理的实现函数
其中main.py为主函数代码，其余各个部分的功能实现位于utils文件夹下
main.py中实现了从大模型意图识别到小车定位与运动以及机械臂和机械手执行任务的上层代码

本项目部分功能基于ROS2开发
dofbot_moveit文件夹下主要存放ROS2实现逆运动学服务器的文件

要修改代码的话记得修改完代码ctrl+s后，在命令行中运行colcon build进行项目编译（注意确保命令行此时在ros2_ws下），待编译完成后输入
source ./install/setup.sh
ros2 run dofbot_garbage_yolov5 block_cls


要运行机械臂的话首先开一个终端按如下顺序运行：
cd   /root/ascend-devkit/src/E2E-Sample/ros2_robot_arm/ros2_ws
source ./install/setup.sh
ros2 run dofbot_moveit dofbot_server
再运行主体代码：命令行中
cd   /root/ascend-devkit/src/E2E-Sample/ros2_robot_arm/ros2_ws
source ./install/setup.sh
ros2 run dofbot_garbage_yolov5 main_only_find（这个调用的是只是测试机械臂抓取代码main_only_find.py，因此没有与小车联动）
ros2 run dofbot_garbage_yolov5 block_cls（这个调用的是只是测试机械臂抓取代码main_only_find.py，因此没有与小车联动）
这里注意main_only_find和block_cls命名都是在dofbot_garbage_yolov5/setup.py中才能修改关联相关代码


