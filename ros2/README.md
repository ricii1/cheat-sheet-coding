# ROS 2
## Daftar Isi
- [ROS 2](#ros-2)
  - [Daftar Isi](#daftar-isi)
  - [Membuat Package](#membuat-package)
  - [Build](#build)
  - [Node](#node)
  - [Topic](#topic)
  - [Service](#service)
  - [Action](#action)
  - [Param](#param)
  - [TF](#tf)
  - [ROS2 Bag](#ros2-bag)
    - [Merekam ROS2 Bag](#merekam-ros2-bag)
    - [Play ROS2 Bag](#play-ros2-bag)
    - [Info ROS2 Bag](#info-ros2-bag)
  - [Logging \& Debugging](#logging--debugging)

## Membuat Package
- Python
  ```
  ros2 pkg create --build-type ament_python <package_name>
  ```
- C++
  ```
  ros2 pkg create --build-type ament_cmake <package_name>
  ```
## Build
- Colcon build
  ```
  colcon build
  ```
- Colcon build spesific package
  ```
  colcon build --packages-select <package_name>
  ```
- Colcon build symlink install
  ```
  colcon build --symlink-install
  ```
## Node
- Menjalankan node dari package
  ```
  ros2 run <package_name> <executable_name>
  ```
- Melihat daftar node berjalan
  ```
  ros2 node list
  ```
- Melihat informasi detail tentang suatu node
  ```
  ros2 node info <node_name>
  ```
## Topic
- Melihat topic
  ```
  ros2 topic list
  ```
- Mendengar isi topic
  ```
  ros2 topic echo <nama_topic>
  ```
- Mendengar topic dengan bagian tertentu
  ```
  ros2 topic echo <nama_topic> --field <nama_bagian>
  ```
  contoh: 
  ```
  ros2 topic echo /odom --field pose.pose.position
  ```
- Melihat frekuensi topic
  ```
  ros2 topic hz <nama_topic>
  ```
- Melihat informasi topic
  ```
  ros2 topic info <nama_topic>
  ```
## Service
- Melihat daftar service
  ```
  ros2 service list
  ```
- Melihat tipe service
  ```
  ros2 service type <nama_service>
  ```
## Action
- Melihat daftar action
  ```
  ros2 action list
  ```
- Melihat tipe action
  ```
  ros2 action type <nama_action>
  ```
## Param
- Melihat daftar parameter dari sebuah node
  ```
  ros2 param list
  ```
- Melihat nilai parameter tertentu
  ```
  ros2 param get <nama_node> <nama_parameter>
  ```
- Mengubah nilai parameter pada node yang mendukung dynamic reconfigure
  ```
  ros2 param set <nama_node> <nama_parameter> <value>
  ```
## TF
- Melihat tf frames
  ```
  ros2 run tf2_tools view_frames
  ```
- Melihat transformasi antara 2 frame
  ```
  ros2 run tf2_ros tf2_echo [source_frame] [target_frame]
  ```
## ROS2 Bag
### Merekam ROS2 Bag
- Merekam Rosbag dengan spesific topic
  ```
  ros2 bag record <nama_topic>
  ```
- Merekam seluruh Rosbag
  ```
  ros2 bag record -a
  ```
### Play ROS2 Bag
- Memulai Rosbag
  ```
  ros2 bag play <nama_file_rosbag>
  ```
- Loop Rosbag
  ```
  ros2 bag play <nama_file_rosbag> --loop
  ```
### Info ROS2 Bag
- Mendapatkan info Rosbag
  ```
  ros2 bag info <nama_file_rosbag>
  ```
## Logging & Debugging
- Menampilkan log 
  ```
  ros2 topic echo /rosout
  ```
- Menjalankan ROS 2 dengan level log tertentu
  ```
  export RCUTILS_LOGGING_SEVERITY=DEBUG
  ros2 run <nama_package> <nama_node>
  ```