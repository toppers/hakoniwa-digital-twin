#!/bin/bash

if [ $# -ne 1 ]
then
    echo "Usage: $0 {real|sim|ar}"
    exit 1
fi

ACT_MODE=$1
CUSTOM_JSON=`pwd`/digital/config/${ACT_MODE}/custom.json

HAKONIWA_RADIO_CTRL_PATH=../hakoniwa-px4sim/drone_api/sample
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
function signal_handler() {
    echo "SIGINT"
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
    if [ $ACT_MODE = "ar" ]
    then
        bash drone-app.bash ../../hakoniwa-unity-drone-model config/rc &
    else
        bash drone-app.bash ../../hakoniwa-unity-drone-model config/api_twin &
    fi
    HAKO_PX4SIM_PID=$!
    cd $CURR_DIR
}
function activate_shmproxy()
{
    CURR_DIR=`pwd`
    cd $HAKONIWA_SHMPROXY_PATH
    ./cmake-build/shm-proxy/shm-proxy ShmProxy ${CUSTOM_JSON} 20 &
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
function activate_ardemo()
{
    CURR_DIR=`pwd`
    cd $HAKONIWA_VREAL_PATH
    bash plugin/activate_app.bash ar-demo &
    HAKO_VREAL_PID=$!
    cd $CURR_DIR
}
function adjust_initial_pos()
{
    CONFIG_PATH=`pwd`/digital/config/ar/xr_config.json
    AR_DEVICE_IPADDR=`jq -r '.server_url' digital/config/ar/xr_config.json | awk -F: '{print $1}'`
    CURR_DIR=`pwd`
    cd $HAKONIWA_VREAL_PATH
    echo "access: ${AR_DEVICE_IPADDR}:38528"
    python ${HAKONIWA_VREAL_PATH}/utils/xr_origin_tuning.py --input joystick ${CONFIG_PATH} ${AR_DEVICE_IPADDR}:38528
    cd $CURR_DIR
}

function radio_control()
{
    CURR_DIR=`pwd`
    cd $HAKONIWA_RADIO_CTRL_PATH
    if [ -f ~/myenv/bin/activate  ]
    then
        source ~/myenv/bin/activate
        PYTHON_BIN=python3.12
    else
        PYTHON_BIN=python
    fi
    ${PYTHON_BIN} rc.py ../../../hakoniwa-unity-drone-model/custom.json
    cd $CURR_DIR
}

activate_px4sim

if [ "${ACT_MODE}" = "sim" ]
then
    sleep 1
    activate_vreal
fi

sleep 1
if [ "${ACT_MODE}" == "ar" ]
then
    activate_ardemo
    #echo "Start Unity"
    #read
else
    activate_shmproxy
    sleep 1
    echo "Start Unity"
    read
fi


echo "START ADJUST INITIAL POSITION"
adjust_initial_pos

echo "START RADIO CONTROL"
radio_control

echo "START"
while true; do
    echo "Press ENTER to stop..."
    read input
    if [ -z "$input" ]; then
        kill_process
        break
    fi
done
