#Guide for building own docker container under Ubuntu 16.04

The original docker container has some issues if host machine is under Ubuntu 16.04.

Here is guide how to build own docker if you use Ubuntu 16.04.

1. Clone repository from lab gitlab and checkout to r3.0.0 branch

```
git clone http://cordelia.university.innopolis.ru/kamaz/apollo.git
git checkout r3.0.0
```

2. Change directory to apollo/docker/build and run:
 `./build_dev.sh dev.x86_64.dockerfile`
3. After building of container is finished, change directory to apollo and run:
`./docker/scripts/dev_start.sh -l -t dev-x86_64-20180708_2029` (You need to use name of own container)
4. Go into the container using the following command:
`./docker/scripts/dev_start.sh` 
5. Build apollo project:
`./apollo.sh build`
6. Run apollo core:
`./scripts/bootstrap.sh`

To stop the apollo core:
`./scripts/bootstrap.sh stop`

To run the preceprion module:
`./apollo.sh build_opt_gpu`
`./scripts/perception_offline_visualizer.sh`

To stop the perseption module:
`./scripts/perception_offline_visualizer.sh stop`
