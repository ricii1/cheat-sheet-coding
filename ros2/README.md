# ROS 2
## TF
- Melihat tf frames
  ```
  ros2 run tf2_tools view_frames
  ```
- Melihat transformasi antara 2 frame
  ```
  ros2 run tf2_ros tf2_echo [source_frame] [target_frame]
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
