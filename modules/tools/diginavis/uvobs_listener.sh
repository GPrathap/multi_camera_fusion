###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${DIR}/../../../scripts/apollo_base.sh"

function start() {
    LOG="${APOLLO_ROOT_DIR}/data/log/uvobs-listener-node.out"
    CMD="python modules/tools/diginavis/uvobs-listener-node.py"
    NUM_PROCESSES_G2L="$(pgrep -c -f uvobs-listener-node)"
    # echo "NUM_PROCESSES" ${NUM_PROCESSES_G2L}
    if [ "${NUM_PROCESSES_G2L}" -eq 0 ]; then
        eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
    # python modules/tools/diginavis/uvobs-listener-node.py
}

function stop() {
    pkill -9 -f uvobs-listener-node
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