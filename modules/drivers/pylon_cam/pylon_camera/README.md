
## pylon_camera
pylon_camera (Basler Cameras) ROS nodelet

### Topics

* /apollo/sensor/camera/perception/image_front_left_side --> sensor_msgs/Image
* /apollo/sensor/camera/perception/image_front_right_side --> sensor_msgs/Image
* /apollo/perception/obstacle/camera --> sensor_msgs/Image
* /apollo/sensor/camera/perception/image_front_left_side/camera_info --> sensor_msgs/CameraInfo
* /apollo/sensor/camera/perception/image_front_right_side/camera_info --> sensor_msgs/CameraInfo
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

### pylon_camera
**Setting the device id for a given camera**

```bash
/home/tmp/ros/lib/pylon_camera/pylon_camera_write_device_user_id_to_camera <name_of_the_camera>
e.g., /home/tmp/ros/lib/pylon_camera/pylon_camera_write_device_user_id_to_camera traffic_left
```

**Launch**

```bash
roslaunch pylon_camera start_pylon_camera.launch
# or
bash /apollo/scripts/pylon_camera.sh
```

**Parameters, topics and nodes (Here it shows parameters, topics and nodes corresponding to the two cameras).**

 ```bash
PARAMETERS
 * /camera_front_left_side/sensor_camera_front_left_side/auto_exposure_upper_limit: 2000000.0
 * /camera_front_left_side/sensor_camera_front_left_side/camera_frame: pylon_camera
 * /camera_front_left_side/sensor_camera_front_left_side/device_user_id: obstacle_left
 * /camera_front_left_side/sensor_camera_front_left_side/exposure_search_timeout: 5.0
 * /camera_front_left_side/sensor_camera_front_left_side/frame_rate: 20
 * /camera_front_left_side/sensor_camera_front_left_side/shutter_mode: 
 * /camera_front_left_side/sensor_camera_front_left_side/trigger_fps: 20
 * /camera_front_left_side/sensor_camera_front_left_side/trigger_internal: 0
 * /camera_front_right_side/sensor_camera_front_right_side/auto_exposure_upper_limit: 2000000.0
 * /camera_front_right_side/sensor_camera_front_right_side/camera_frame: pylon_camera
 * /camera_front_right_side/sensor_camera_front_right_side/device_user_id: obstacle_right
 * /camera_front_right_side/sensor_camera_front_right_side/exposure_search_timeout: 5.0
 * /camera_front_right_side/sensor_camera_front_right_side/frame_rate: 20
 * /camera_front_right_side/sensor_camera_front_right_side/shutter_mode: 
 * /camera_front_right_side/sensor_camera_front_right_side/trigger_fps: 20
 * /camera_front_right_side/sensor_camera_front_right_side/trigger_internal: 0
 * /rosdistro: indigo
 * /rosversion: 1.11.21

NODES
  /camera_front_right_side/
    camera_nodelet_manager (nodelet/nodelet)
    sensor_camera_front_right_side (nodelet/nodelet)
  /camera_front_left_side/
    camera_nodelet_manager (nodelet/nodelet)
    sensor_camera_front_left_side (nodelet/nodelet)
```
