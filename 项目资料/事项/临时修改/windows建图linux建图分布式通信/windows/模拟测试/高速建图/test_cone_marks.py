import importlib.machinery, importlib.util, json, sys
from pathlib import Path
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid

run = Path(sys.argv[1])
loader = importlib.machinery.SourceFileLoader('cm', r'C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\cone_marks')
spec = importlib.util.spec_from_loader(loader.name, loader)
cm = importlib.util.module_from_spec(spec)
loader.exec_module(cm)

c = json.loads((run / 'complete.json').read_text(encoding='utf-8'))
w, h, res = c['map_width'], c['map_height'], c['map_resolution']
raw = np.fromfile(run / 'map.raw', dtype=np.int8)
assert raw.size == w * h, (raw.size, w * h)
msg = OccupancyGrid()
msg.header.frame_id = 'map'
msg.info.width, msg.info.height, msg.info.resolution = w, h, res
msg.info.origin.position.x, msg.info.origin.position.y = c['map_origin']['x'], c['map_origin']['y']
msg.data = raw.tolist()

rclpy.init()
node = cm.ConeMarks()
got = []
node.pub.publish = lambda arr: got.append(arr)
node.on_map(msg)
node.publish()
node.publish()          # 同一张图不重复算
arr = got[0]
print('publish 次数', len(got), '；markers', len(arr.markers), '（第一个是 DELETEALL：', arr.markers[0].action, '）')
m = arr.markers[1]
print('第一个锥桶', round(m.pose.position.x, 2), round(m.pose.position.y, 2), 'scale', m.scale.x, 'color', m.color.r, m.color.a)
solid = json.loads((run / 'solid.json').read_text(encoding='utf-8'))
print('存图后描黑 solid.json 锥桶数', solid.get('cones'))
node.destroy_node(); rclpy.shutdown()
