#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

function start() {
    LOG="${APOLLO_ROOT_DIR}/data/log/geo2loc.out"
    CMD="roslaunch geo2loc geo2loc_rtk.launch"
    NUM_PROCESSES_G2L="$(pgrep -c -f "geo2loc.launch")"
    #echo "NUM_PROCESSES" ${NUM_PROCESSES_G2L}
    if [ "${NUM_PROCESSES_G2L}" -eq 0 ]; then
        #echo "Start geo2loc"
        eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function stop() {
    pkill -9 -f geo2loc
    pkill -9 -f nmea_serial_driver
    pkill -9 -f mtnode.py
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
