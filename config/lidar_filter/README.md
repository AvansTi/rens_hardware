# Lidar Filtering

Run the laser filtering node:

```bash
ros2 run laser_filters scan_to_scan_filter_chain \
  --ros-args -p scan_topic:=scan \
  -p target_frame:=base_link \
  --params-file /home/rens/rens_hardware/config/lidar_filter/lidar_filters.yaml
```
