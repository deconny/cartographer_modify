# cartographer_modify

# Build
```bash
colcon build --merge-install --packages-up-to cartographer_ros
```

# Run
```bash
source install/setup.bash
ros2 launch cartographer_ros demo_backpack_2d_localization.launch.py
```

默认模式是 mapping，可以通过调用服务切换模式。

# 切换模式
可以通过调用 `switch_mode` 服务来切换模式。支持的模式有 `mapping` 和 `localization`。

## 切换到 Mapping 模式
```bash
ros2 service call /switch_mode cartographer_ros_msgs/srv/SwitchMode "{mode: 'mapping'}"
```

## 切换到 Localization 模式
```bash
ros2 service call /switch_mode cartographer_ros_msgs/srv/SwitchMode "{mode: 'localization', map_filename: 'path/to/map.pbstream'}"
```

注意：`map_filename` 必须是一个有效的 `.pbstream` 文件路径。