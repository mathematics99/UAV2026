#!/usr/bin/env python3
"""Symmetric linear funnel from the historical funnel_channel_map.py, without forest."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def config(gap):
    return dict(entrance_width_m=3.0, throat_width_m=gap, half_length_m=3.0,
                wall_outer_y_m=12.0, wall_z_min_m=0.0, wall_z_max_m=2.5,
                resolution_m=0.1, map_size_m=[50.0,25.0,2.0],
                profile='w(x)=gap/2+(1.5-gap/2)*abs(x)/3, -3<=x<=3',
                extra_obstacles=0, seed=1,
                note='Single linear contraction to x=0, followed by its linear exit; historical funnel geometry. Symmetric inward wall faces.')


def points(c):
    parts=[]
    profile=[]
    zs=np.linspace(0,c['wall_z_max_m'],26)
    for x in np.linspace(-3,3,61):
        w=c['throat_width_m']/2+(1.5-c['throat_width_m']/2)*abs(x)/3
        profile.append((x,2*w,w,-w))
        ys=w+np.arange(int(np.floor((12-w)/.1+1e-8))+1)*.1
        yy,zz=np.meshgrid(ys,zs,indexing='ij')
        p=np.column_stack((np.full(yy.size,x),yy.ravel(),zz.ravel()))
        parts.extend((p,p*np.array([1,-1,1])))
    return np.vstack(parts).astype('<f4'),profile


def main():
    p=argparse.ArgumentParser();p.add_argument('--gap',type=float,required=True);p.add_argument('--out',required=True)
    a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    c=config(a.gap);arr,profile=points(c)
    (out/'map_config.json').write_text(json.dumps(c,indent=2))
    np.save(out/'wall_points.npy',arr)
    with (out/'wall_profile.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['x_m','width_m','upper_inner_y_m','lower_inner_y_m']);w.writerows(profile)
    rclpy.init();n=Node('section53_linear_channel')
    pub=n.create_publisher(PointCloud2,'/map_generator/global_cloud',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    msg=PointCloud2();msg.header.frame_id='world';msg.height=1;msg.width=len(arr)
    msg.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate('xyz')]
    msg.point_step=12;msg.row_step=12*len(arr);msg.is_dense=True;msg.data=arr.tobytes()
    def publish():
        msg.header.stamp=n.get_clock().now().to_msg();pub.publish(msg)
    n.create_timer(1.0,publish);publish()
    print(f'wall-only linear channel: {c}, points={len(arr)}',flush=True)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
