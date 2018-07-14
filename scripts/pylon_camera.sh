#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

function start() {
    LOG="${APOLLO_ROOT_DIR}/data/log/pylon_camera.out"
    CMD="roslaunch pylon_camera start_pylon_camera.launch"
    NUM_PROCESSES="$(pgrep -c -f "camera_nodelet_manager")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
       eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function stop() {
    pkill -9 -f pylon_camera
    pkill -9 -f camera_nodelet_manager
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
