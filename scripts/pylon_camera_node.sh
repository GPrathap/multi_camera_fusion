#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

function start() {
    LOG="${APOLLO_ROOT_DIR}/data/log/pylon_camera.out"
    CMD="roslaunch pylon_camera pylon_camera_node.launch"
    NUM_PROCESSES="$(pgrep -c -f "pylon_camera_node.launch")"
    echo $NUM_PROCESSES
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        echo "Start pylon camera"
       eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function stop() {
    pkill -9 -f pylon_camera_node
}

# run command_name module_name
function run() {
    case $1 in
        start)
            start
            ;;
        stop)
            stop
            ;;
        *)
    esac
}

run "$1"
