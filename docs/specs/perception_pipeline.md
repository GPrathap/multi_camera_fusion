The image processing pipeline consist from the following steps:
1. The perception module initializates from a DAG config file which describes loaded subnodes. [Here](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/conf/dag_camera_obstacle_vis.config) is an example which we use for oue KIA experiments. As we can see there, we load a CameraProcessSubnode and VisualizationSubnode and connect them by edge.
2. The camera process subnode get a [callback](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/camera_process_subnode.cc#L104) with Image message.
3. The camera process subnode calls some subprocess:
* [detection by CNN](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/camera_process_subnode.cc#L137)
* [convertions coordinates from 2D to 3D](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/camera_process_subnode.cc#L168)
* [update tracker objects](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/camera_process_subnode.cc#L178)
* [filtering positions and velocity objects](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/camera_process_subnode.cc#L196)
* [publish detected objects](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/camera_process_subnode.cc#L214)

I think there are several ways to implement N cameras support.

1. To add additional section in camera subnode into DAG file which will describe all cameras connected with the subnode. So one camera subnode will handle several cameras. In this case, we have to add additional informtation in Image callback to give the algorithm an information about camera position and orientation. The disadvantage of this approach is that we process images from all cameras consistently. So it will cause big delay. The advantage is that we load into GPU memory only one copy of CNN.

2. Create several camera subnodes into the DAG file. However, we don't have to use different source code for each subnode. In this case, we also have to add a section which will describe a camera confuguration for each subnode. The advantage is that we can process images in parallel (ideally on different GPU). The disadvantege is that we have to keep several copies of CNN in GPU. 

3. Combine both approaches. Several camera subnodes which can handles several cameras each.

Anyway, the main my idea that we should configure DAG file to describe used cameras without coping of source code.