# mavros-bridge
Interface with with PX4 mavros 


##
```bash
sudo apt install ros-humble-mavros
sudo ./ros2/src/mavros_bridge/install_geolib.sh
rosdep install --from-paths src/mavros_bridge --ignore-src -r -y
colcon build --packages-select mavros_bridge
source install/setup.bash
```

```bash
ros2 launch mavros_bridge mavros_bridge.launch.py
```

```
ros2 launch zed_ros2_minimal zed_camera.launch.py camera_model:=zedm 

ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed_node/rgb/image_rect_color \
    depth_topic:=/zed_node/depth/depth_registered \
    camera_info_topic:=/zed_node/rgb/camera_info \
    frame_id:=zed_camera_link \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/zed_node/imu/data \
    rviz:=true
```