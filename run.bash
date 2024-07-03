#!/bin/bash

HAKONIWA_PX4SIM_PATH=../hakoniwa-px4sim/hakoniwa
HAKONIWA_SHMPROXY_PATH=bridge/virtual
HAKONIWA_VREAL_PATH=../hakoniwa-unity-drone-model

HAKO_PX4SIM_PID=
HAKO_SHMPROXY_PID=
HAKO_VREAL_PID=
function kill_process()
{
    echo "trapped"
    if [ ! -z "$HAKO_SHMPROXY_PID" ]
    then
        echo "KILLING: shmproxy $HAKO_SHMPROXY_PID"
        kill -s TERM "$HAKO_SHMPROXY_PID" || echo "Failed to kill shmproxy"
    fi

    if [ ! -z "$HAKO_VREAL_PID" ]
    then
        echo "KILLING: vreal $HAKO_VREAL_PID"
        kill -s TERM "$HAKO_VREAL_PID" || echo "Failed to kill vreal"
    fi
    if [ ! -z "$HAKO_PX4SIM_PID" ]
    then
        echo "KILLING: px4sim $HAKO_PX4SIM_PID"
        kill -s TERM "$HAKO_PX4SIM_PID" || echo "Failed to kill px4sim"
    fi

    exit 0
}

trap signal_handler SIGINT

export PYTHONPATH="/usr/local/lib/hakoniwa:$PYTHONPATH"
export PYTHONPATH="/usr/local/lib/hakoniwa/py:$PYTHONPATH"
export PATH="/usr/local/bin/hakoniwa:${PATH}"
export LD_LIBRARY_PATH="/usr/local/lib/hakoniwa:${LD_LIBRARY_PATH}"
export DYLD_LIBRARY_PATH="/usr/local/lib/hakoniwa:${DYLD_LIBRARY_PATH}"


function activate_px4sim()
{
    CURR_DIR=`pwd`
    cd $HAKONIWA_PX4SIM_PATH
    bash drone-app.bash ../../hakoniwa-unity-drone-model config/api_twin &
    HAKO_PX4SIM_PID=$!
    cd $CURR_DIR
}
function activate_shmproxy()
{
    CURR_DIR=`pwd`
    cd $HAKONIWA_SHMPROXY_PATH
    ./cmake-build/shm-proxy/shm-proxy ShmProxy ../../digital/config/custom.json 20 &
    HAKO_SHMPROXY_PID=$!
    cd $CURR_DIR
}
function activate_vreal()
{
    CURR_DIR=`pwd`
    cd $HAKONIWA_VREAL_PATH
    bash plugin/activate_app.bash TwinReal &
    HAKO_VREAL_PID=$!
    cd $CURR_DIR
}

activate_px4sim

sleep 1
activate_vreal

sleep 1
activate_shmproxy

echo "START"
while true; do
    echo "Press ENTER to stop..."
    read input
    if [ -z "$input" ]; then
        kill_process
        break
    fi
done