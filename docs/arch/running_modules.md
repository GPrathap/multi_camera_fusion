# Module running

For running the apollo modules there are some scripts into `/apollo/scripts' folder.

For instance, for running `perception` module you can use `/apollo/scripts/perception.sh`:
```
source "${DIR}/apollo_base.sh"
# run function from apollo_base.sh
# run command_name module_name
run perception "$@"
```
which runs the preception module with flags described in `/apollo/modules/perception/conf/perception.conf`

If you want change some flags in compare with `perception.conf` you can use two ways.

The first way is to add needed flags directly in the run command like in `/apollo/scripts/perception_offline_visualizer.sh`
```
run perception "$@" --dag_config_path=/apollo/modules/perception/conf/dag_camera_obstacle_vis.config \
--alsologtostderr=1 --v=4 
```
In this case you set there flags directly.

Another way is to point the run command at certain flag file like in `/apollo/scripts/perception_lowcost_vis.sh`
```
run perception "$@" --flagfile=/apollo/modules/perception/conf/perception_lowcost_vis.conf
```

Each module contains several subnodes. You can define necessary subnodes into `dag_.config' file.
Here is example of dag.config with two subnodes and link between them:
```
subnode_config {
    # Camera node. publish pb with non-zero int
    subnodes {
        id: 3
        name: "CameraProcessSubnode"
        reserve: "device_id:camera;pb_obj:1;pb_ln_msk_:0;"
        type: SUBNODE_IN
    }
    # Visualization node with OpenGL
    subnodes {
        id: 41
        name: "VisualizationSubnode"
        reserve: "vis_driven_event_id:1008;camera_event_id:1008"
    }
}

###################################################################
# Define all edges which link nodes.
edge_config {
		# CameraProcessSubnode -> VisualizationSubnode
    edges {
        id: 108
        from_node: 3
        to_node: 41
        events {
            id: 1008
            name: "camera_visualization"
        }
    }
}

# Shared Data
data_config {
    datas {
        id: 5
        name: "CameraObjectData"
    }
    datas {
        id: 7
        name: "CameraSharedData"
    }
}
```

