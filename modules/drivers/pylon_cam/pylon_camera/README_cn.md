
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


