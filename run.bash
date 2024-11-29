#!/bin/bash

CUSTOM_JSON=`pwd`/digital/config/ar/custom.json

HAKONIWA_WEB_SERVER_PATH=`pwd`/../hakoniwa-webserver
HAKONIWA_AR_BRIDGE_PATH=`pwd`/../si2024/hakoniwa-ar-bridge
HAKONIWA_PX4SIM_PATH=`pwd`/../hakoniwa-px4sim/hakoniwa
HAKONIWA_SHMPROXY_PATH=bridge/virtual
HAKONIWA_UNITY_APL_PATH=`pwd`/../si2024/hakoniwa-unity-drone-model/DTwinAppleSilicon
export PYTHONPATH="/usr/local/lib/hakoniwa:$PYTHONPATH"
export PYTHONPATH="/usr/local/lib/hakoniwa/py:$PYTHONPATH"
export PATH="/usr/local/bin/hakoniwa:${PATH}"
export LD_LIBRARY_PATH="/usr/local/lib/hakoniwa:${LD_LIBRARY_PATH}"
export DYLD_LIBRARY_PATH="/usr/local/lib/hakoniwa:${DYLD_LIBRARY_PATH}"

if [ -f ~/myenv/bin/activate  ]
then
    source ~/myenv/bin/activate
    PYTHON_BIN=python3.12
else
    PYTHON_BIN=python
fi


HAKO_PX4SIM_PID=
HAKO_SHMPROXY_PID=
HAKO_UNITY_PID=
HAKO_WEBSRV_PID=
function kill_process()
{
    echo "trapped"
    if [ ! -z "$HAKO_SHMPROXY_PID" ]
    then
        echo "KILLING: shmproxy $HAKO_SHMPROXY_PID"
        kill -s TERM "$HAKO_SHMPROXY_PID" || echo "Failed to kill shmproxy"
    fi

    if [ ! -z "$HAKO_UNITY_PID" ]
    then
        echo "KILLING: unity $HAKO_UNITY_PID"
        kill -s TERM "$HAKO_UNITY_PID" || echo "Failed to kill unity"
    fi
    if [ ! -z "$HAKO_WEBSRV_PID" ]
    then
        echo "KILLING: web server $HAKO_WEBSRV_PID"
        kill -s TERM "$HAKO_WEBSRV_PID" || echo "Failed to kill web server"
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

function activate_px4sim()
{
    CURR_DIR=`pwd`
    cd $HAKONIWA_PX4SIM_PATH
    bash drone-app.bash ${HAKONIWA_UNITY_APL_PATH} config/rc &
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
function activate_unity()
{
    CURR_DIR=`pwd`
    cd $HAKONIWA_UNITY_APL_PATH
    cd ..
    bash plugin/activate_app.bash DTwinAppleSilicon &
    HAKO_UNITY_PID=$!
    cd $CURR_DIR
}

function activate_web_server()
{
    CURR_DIR=`pwd`
    cd $HAKONIWA_WEB_SERVER_PATH
    ${PYTHON_BIN} server/main.py --asset_name WebServer --config_path config/twin-custom.json --delta_time_usec 20000 &
    HAKO_WEBSRV_PID=$!
    cd $CURR_DIR
}

function activate_ar_bridge()
{
    CURR_DIR=`pwd`
    CONFIG_PATH=`pwd`/digital/config/ar/ar_bridge_config.json 
    cd $HAKONIWA_AR_BRIDGE_PATH
    ${PYTHON_BIN} asset_lib/main.py --config ${CONFIG_PATH}
    cd $CURR_DIR
}

activate_px4sim

sleep 1

activate_unity

sleep 1

activate_shmproxy


sleep 5

hako-cmd start

sleep 1

activate_web_server

activate_ar_bridge

kill_process
