import collections
import importlib.machinery
import importlib.util
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from builtin_interfaces.msg import Time

scripts=Path(__file__).resolve().parents[2]/'scripts'
sys.path.insert(0,str(scripts))
loader=importlib.machinery.SourceFileLoader('factor_node_test',str(scripts/'cone_factor_graph_node'))
spec=importlib.util.spec_from_loader(loader.name,loader)
module=importlib.util.module_from_spec(spec)
loader.exec_module(module)

class QueueTests(unittest.TestCase):
    def test_scan_waits_for_tf_then_processes_once(self):
        class FakeTf:
            ready=False
            def can_transform(self,*args,**kwargs): return self.ready
        tf=FakeTf(); processed=[]
        obj=SimpleNamespace(scan_queue=collections.deque(),tf=tf,odom_frame='odom_pc',base_frame='base_pc',
                            tf_failures=0,on_scan=lambda msg:processed.append(msg))
        msg=SimpleNamespace(header=SimpleNamespace(stamp=Time(sec=1,nanosec=0),frame_id='laser_pc'))
        obj.scan_queue.append((time.monotonic(),msg))
        module.ConeFactorGraphNode.drain_scans(obj)
        self.assertEqual(len(obj.scan_queue),1)
        self.assertEqual(processed,[])
        tf.ready=True
        module.ConeFactorGraphNode.drain_scans(obj)
        self.assertEqual(processed,[msg])
        self.assertEqual(len(obj.scan_queue),0)

if __name__=='__main__':unittest.main()
