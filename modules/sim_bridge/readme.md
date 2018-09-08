# ROS bridge module  

Now this module has two type: Unity bridge and ROS localization bridge

## Unity simulation bridge

This module converts regular ROS messages from a simulator to apollo messages and vice versa.

To run the module:
```
./apollo.sh sim_bridge.sh
```

## ROS localization bridge

This module converts regular ROS odometry messages to apollo localization messages.

To run the module:
```
./apollo.sh ros_localization_bridge.sh
```

