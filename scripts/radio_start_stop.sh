#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

source /home/tmp/ros/setup.bash

function start() {
    LOG="${APOLLO_ROOT_DIR}/data/log/radio_start_stop.out"
    CMD="roslaunch radio_start_stop radio.launch"
    NUM_PROCESSES="$(pgrep -c -f "radio.launch")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function stop() {
    # pkill -9 -f nmea_serial_driver && pkill -9 -f mtnode.py && pkill -9 -f ekf_localization_node && pkill -9 -f geo2loc 
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
