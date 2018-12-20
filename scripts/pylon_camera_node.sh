#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

function start_front_camera() {
    LOG="${APOLLO_ROOT_DIR}/data/log/start_front_camera.out"
    CMD="roslaunch pylon_camera pylon_camera_node.launch"
    NUM_PROCESSES="$(pgrep -c -f "pylon_camera_node.launch")"
    echo $NUM_PROCESSES
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        echo "Start front camera "
       eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function start_front_and_side_cameras() {
    LOG="${APOLLO_ROOT_DIR}/data/log/start_front_and_side_cameras.out"
    CMD="roslaunch pylon_camera start_pylon_camera.launch"
    NUM_PROCESSES="$(pgrep -c -f "start_pylon_camera.launch")"
    echo $NUM_PROCESSES
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        echo "Start front camera and front side cameras"
       eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function stop_front_camera() {
    pkill -9 -f pylon_camera_node
}

function stop_front_and_side_cameras() {
    pkill -9 -f  start_pylon_camera
}

# run command_name module_name
function run() {
    case $1 in
        start_front_camera)
            start_front_camera
            ;;
        start_front_and_side_cameras)
            start_front_and_side_cameras
            ;;
        stop_front_camera)
            stop_front_camera
            ;;
        stop_front_and_side_cameras)
            stop_front_camera
            ;;
        *)
    esac
}

run "$1"
