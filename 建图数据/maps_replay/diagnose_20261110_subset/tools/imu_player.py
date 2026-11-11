import argparse, sqlite3, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan, Imu

ap = argparse.ArgumentParser()
ap.add_argument('bag')
ap.add_argument('--rate', type=float, default=1.0)
ap.add_argument('--track')
ap.add_argument('--odom-lag')
args = ap.parse_args()
db = sqlite3.connect('file:' + args.bag.replace('\\', '/') + '?mode=ro', uri=True)
ids = {name: ident for ident, name in db.execute('select id,name from topics')}
rclpy.init()
node = Node('imu_only_bag_player')
scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
imu_pub = node.create_publisher(Imu, '/IMU_data', qos_profile_sensor_data)
time.sleep(2)
start = None
counts = {'scan': 0, 'imu': 0}
for topic, stamp, data in db.execute('select topic_id,timestamp,data from messages where topic_id in (?,?) order by timestamp',
                                     (ids['/scan'], ids['/IMU_data'])):
    t = stamp * 1e-9
    if start is None:
        start = (t, time.monotonic())
    delay = start[1] + (t - start[0]) / args.rate - time.monotonic()
    if delay > 0:
        time.sleep(delay)
    if topic == ids['/scan']:
        scan_pub.publish(deserialize_message(data, LaserScan)); counts['scan'] += 1
    else:
        imu_pub.publish(deserialize_message(data, Imu)); counts['imu'] += 1
print('已发送', counts, flush=True)
time.sleep(1)
node.destroy_node()
rclpy.shutdown()
