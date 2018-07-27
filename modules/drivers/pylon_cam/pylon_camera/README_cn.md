
## pylon_camera
pylon_camera (Basler Cameras相)设备实ROS nodelet封装，提供图像采集及发布的功能。本驱动中使用了一台长焦相机和一台短焦相机。

### Topics

* /apollo/sensor/camera/traffic/image_long --> sensor_msgs/Image
* /apollo/sensor/camera/traffic/image_short --> sensor_msgs/Image
* /apollo/perception/obstacle/camera --> sensor_msgs/Image
* /apollo/sensor/camera/traffic/image_long/camera_info --> sensor_msgs/CameraInfo
* /apollo/sensor/camera/traffic/image_short/camera_info --> sensor_msgs/CameraInfo
* /apollo/perception/obstacle/camera/camera_info --> sensor_msgs/CameraInfo


```bash
# in dev docker
# to install pylon sdk
cd /apollo/modules/drivers/pylon_cam/pylon_camera/sdk
bash install_pylon_camera_sdk.sh
# to install pylon camera driver
cd /apollo
bash apollo.sh build_pylon_cam
```

```
Installed Pylon camera driver can be located in the follwoing directory
/apollo/bazel-apollo/external/ros/share/pylon_camera
```


**内参文件**
```xml
## TODO adding camera parameters
```

### pylon_camera
**请先修改并确认launch文件中的参数与实际车辆相对应**

```bash
roslaunch pylon_camera start_pylon_camera.launch
# or
bash /apollo/scripts/pylon_camera.sh
```

**请先修改并确认 Parameters, topics and nodes (Here it shows parameters, topics and nodes corresponding to the two cameras).文件中的参数与实际车辆相对应**

 ```bash
PARAMETERS
 * /camera_long/sensor_camera_traffic_long/auto_exposure_upper_limit: 2000000.0
 * /camera_long/sensor_camera_traffic_long/camera_frame: pylon_camera
 * /camera_long/sensor_camera_traffic_long/device_user_id: obstacle_left
 * /camera_long/sensor_camera_traffic_long/exposure_search_timeout: 5.0
 * /camera_long/sensor_camera_traffic_long/frame_rate: 20
 * /camera_long/sensor_camera_traffic_long/shutter_mode:
 * /camera_long/sensor_camera_traffic_long/trigger_fps: 20
 * /camera_long/sensor_camera_traffic_long/trigger_internal: 0
 * /camera_short/sensor_camera_traffic_short/auto_exposure_upper_limit: 2000000.0
 * /camera_short/sensor_camera_traffic_short/camera_frame: pylon_camera
 * /camera_short/sensor_camera_traffic_short/device_user_id: obstacle_right
 * /camera_short/sensor_camera_traffic_short/exposure_search_timeout: 5.0
 * /camera_short/sensor_camera_traffic_short/frame_rate: 20
 * /camera_short/sensor_camera_traffic_short/shutter_mode:
 * /camera_short/sensor_camera_traffic_short/trigger_fps: 20
 * /camera_short/sensor_camera_traffic_short/trigger_internal: 0
 * /rosdistro: indigo
 * /rosversion: 1.11.21

NODES
  /camera_short/
    camera_nodelet_manager (nodelet/nodelet)
    sensor_camera_traffic_short (nodelet/nodelet)
  /camera_long/
    camera_nodelet_manager (nodelet/nodelet)
    sensor_camera_traffic_long (nodelet/nodelet)
```

**请先修改并确认Setting the device id for a given camera文件中的参数与实际车辆相对应**

```bash
/home/tmp/ros/lib/pylon_camera/pylon_camera_write_device_user_id_to_camera <name_of_the_camera>
e.g., /home/tmp/ros/lib/pylon_camera/pylon_camera_write_device_user_id_to_camera traffic_left
```