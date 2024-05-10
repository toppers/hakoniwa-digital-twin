import hakopy
import hako_pdu
import sys

import sys
sys.path.append('..')
from sensors.workspace.src.infra_sensor_2dlidar.infra_sensor_2dlidar.lidar_filter import InfraSensorLidarFilter
from sensors.workspace.src.infra_sensor_2dlidar.infra_sensor_2dlidar.pos_estimator import InfraSensorPositionEstimater

def my_on_initialize(context):
    return 0

def my_on_reset(context):
    return 0

lidar_filter = None
lidar_pos_estimator = None
pdu_manager = None
def my_on_simulation_step(context):
    global pdu_manager
    global lidar_filter
    global lidar_pos_estimator
    pdu_lidar = pdu_manager.get_pdu('2DLiDAR', 0)
    lidar_data = pdu_lidar.read()
    degrees, values = lidar_filter.filter_ranges(lidar_data['intensities'], lidar_data['ranges'])
    x, y = lidar_pos_estimator.run(degrees, values, 100, 10)
    if x > 0 and y > 0:
        print(f"({x}, {y})")
    return 0


my_callback = {
    'on_initialize': my_on_initialize,
    'on_simulation_step': my_on_simulation_step,
    'on_manual_timing_control': None,
    'on_reset': my_on_reset
}
def main():
    global pdu_manager
    global lidar_filter
    global lidar_pos_estimator
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <asset_name> <config_path> <delta_time_msec>")
        return 1
    
    lidar_filter = InfraSensorLidarFilter(360, 10)
    lidar_pos_estimator = InfraSensorPositionEstimater(0, 10, 1.0, 10, 0.1)

    asset_name = sys.argv[1]
    config_path = sys.argv[2]
    delta_time_usec = int(sys.argv[3]) * 1000
    pdu_manager = hako_pdu.HakoPduManager('/usr/local/lib/hakoniwa/hako_binary/offset', config_path)

    hakopy.conductor_start(delta_time_usec, delta_time_usec)
    ret = hakopy.asset_register(asset_name, config_path, my_callback, delta_time_usec, hakopy.HAKO_ASSET_MODEL_CONTROLLER)
    if ret == False:
        print(f"ERROR: hako_asset_register() returns {ret}.")
        return 1

    ret = hakopy.start()
    print(f"INFO: hako_asset_start() returns {ret}")

    hakopy.conductor_stop()
    return 0

if __name__ == "__main__":
    sys.exit(main())
