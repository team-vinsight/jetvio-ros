#!/usr/bin/env python3
"""Detect & correct D455 emitter-metadata inversion so cuVSLAM gets clean (emitter-off)
infra. Exit 0 when output/infra_1 reads clean, 1 if uncorrectable within MAX_TRIES."""
import subprocess, sys, time
import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

SPLIT='/realsense_splitter_node/output/infra_1'
RAW='/camera/camera/infra1/image_rect_raw'
CAM='/camera/camera'
MAX_TRIES=5

def lapvar(msg):
    a=np.frombuffer(bytes(msg.data),np.uint8).reshape(msg.height,msg.step)[:,:msg.width].astype(np.float32)
    lap=-4*a+np.roll(a,1,0)+np.roll(a,-1,0)+np.roll(a,1,1)+np.roll(a,-1,1)
    return float(lap[1:-1,1:-1].var())

class Probe(Node):
    def __init__(self): super().__init__('emitter_probe')
    def collect(self, topic, n, timeout=8.0):
        vals=[]
        sub=self.create_subscription(Image, topic, lambda m: vals.append(lapvar(m)), qos_profile_sensor_data)
        t0=time.time()
        while len(vals)<n and time.time()-t0<timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.destroy_subscription(sub)
        return vals

def toggle():
    for v in ('false','true'):
        subprocess.run(['ros2','param','set',CAM,'depth_module.emitter_on_off',v], stdout=subprocess.DEVNULL)
        time.sleep(2)

def main():
    rclpy.init(); n=Probe()
    for attempt in range(1, MAX_TRIES+1):
        raw=n.collect(RAW,10); out=n.collect(SPLIT,6)
        if len(raw)<4 or len(out)<3:
            n.get_logger().error('not enough frames - is the perception stack up?'); rclpy.shutdown(); return 1
        raw=np.array(raw); out=np.array(out); mid=(raw.min()+raw.max())/2
        lo=raw[raw<mid].mean(); hi=raw[raw>=mid].mean(); om=out.mean()
        clean = abs(om-lo) < abs(om-hi)
        n.get_logger().info(f'try {attempt}: out={om:.0f} clean_lvl={lo:.0f} dotted_lvl={hi:.0f} -> {"CLEAN" if clean else "DOTTED"}')
        if (hi-lo)/max(lo,1) < 0.15:
            n.get_logger().warn('raw phases barely differ - emitter may not be toggling')
        if clean:
            n.get_logger().info('OK - cuVSLAM is getting clean frames.'); rclpy.shutdown(); return 0
        n.get_logger().warn('inverted - toggling emitter_on_off...'); toggle()
    n.get_logger().error(f'still dotted after {MAX_TRIES} tries - try a camera restart.'); rclpy.shutdown(); return 1

if __name__=='__main__':
    sys.exit(main())
