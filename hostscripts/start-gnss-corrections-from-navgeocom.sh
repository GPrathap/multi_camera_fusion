#!/bin/bash

### The following script is aimed to run the gnss str2str relay and an sshpass via tmux
### GNU Screen inside tmux is used in order to initialize a port after corrections are started.
### That's weird that we need to run screen, but that works. Screen btw can be only executed once while str2str is running
###
### Author: Aidar Gabdullin


# Make sure the following config and credentials are correct
HOST="142.93.207.24"
HOSTUSER="roma"
PASSWORD="qweqwe"
REC_PORT=10100
LOC_PORT=5000

function start() {
    tmux has-session -t gnss-corr
    if [ $? != 0 ]
    then
        tmux new-session -s gnss-corr -n "GNSS" -d

        tmux split-window -h -t gnss-corr:0
        tmux split-window -v -t gnss-corr:0.1
        # tmux send-keys -t gnss-corr:0.0 'stty -F /dev/rtk_corr 230400' C-m
        # tmux send-keys -t gnss-corr:0.0 'screen -S gnss-init -X quit' C-m
        # Connect to ssh tunnel
        tmux send-keys -t gnss-corr:0.0 'sshpass -p '$PASSWORD' ssh -L '$LOC_PORT':localhost:'$REC_PORT' '$HOSTUSER'@'$HOST' -N' C-m
        # Run str2str relay
        #
        #tmux send-keys -t gnss-corr:0.1 '/home/race/soul/rtklib_2.4.2/app/str2str/gcc/str2str -in tcpcli://localhost:5000  -out serial://rtk_corr:230400:8:o:1:off' C-m
	#
        ## GNSS correction RTCM messages from Navigeocom
	# tmux send-keys -t gnss-corr:0.1 '/home/race/soul/rtklib_2.4.2/app/str2str/gcc/str2str -p 55.754452 48.740814 1 -in ntrip://aidgab:qwe123qwe@smartnet.navgeocom.ru:7016/USLN -out serial://rtk_corr:230400:8:o:1:off' C-m
	tmux send-keys -t gnss-corr:0.1 '/home/race/soul/rtklib_2.4.2/app/str2str/gcc/str2str -p 55.754452 48.740814 1 -in ntrip://robolabinno:WiVCLD4hXZ92q8V@smartnet.navgeocom.ru:7016/USLN -out serial://rtk_corr:230400:8:o:1:off' C-m
	## GNSS correction RTCM messages from Raspberry
        #tmux send-keys -t gnss-corr:0.1 '/home/race/soul/rtklib_2.4.2/app/str2str/gcc/str2str -in tcpcli://188.130.155.105:32323 -out file://test-corrections-raspberry' C-m
        sleep 1

        # Run screen to initialize port
        # FIXME: we don't really know how screen affects, but it works
        # setting via stty did not help
        tmux send-keys -t gnss-corr:0.2 'screen -S gnss-init /dev/rtk_corr 230400' C-m
        echo "Initializing dev_corr device. Please wait a while..."
        sleep 1
        # Kill the screen
        tmux send-keys -t gnss-corr:0.2 C-a k y

        tmux select-window -t gnss-corr:0
        echo "GNSS Correction session started"
    else echo "GNSS Correction session already exists"
    fi
}

# Stop function is ca
function stop() {
    screen -S gnss-init -X quit
    tmux kill-session -t gnss-corr
    echo "GNSS Correction session killed"
}

function attach() {
    tmux attach -t gnss-corr
}

function bytes() {
    tmux capture-pane -p -t gnss-corr:0.1
}

# run command_name module_name
function run() {
    usage="$(basename "$0") <command> -- run gnss corrections in the background
    The following commands can be used:

        -h  show this help text
        start       Start the tunnel and a str2str in a detached tmux session
        stop        Terminate tunnel and str2str. Kill the session
        restart     Sequential stop+start
        attach      Attach to tmux session in order to see what happens. Use 'C-b d' to detach
        bytes       See if str2str is receiving bytes from tunnel"
        
    case $1 in
        start)
            start
            ;;
        stop)
            stop
            ;;
        attach)
            attach
            ;;
        restart)
            stop
            start
            ;;
        bytes)
            bytes
            ;;
        *)
            echo "$usage"
            exit
            ;;
    esac    
}

run "$1"
