#!/usr/bin/env python3
"""Exercise actual robot_state_publisher launches and check their static TF output.

Run with the system Python after sourcing ROS and the built package workspace.
This checks middleware/runtime descriptions, not rendering or physical fidelity.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def expected_transforms(path):
    expected = {}
    for joint in ET.parse(path).getroot().findall('joint'):
        if joint.get('type') != 'fixed':
            raise ValueError('This test is intended for the fixed visual descriptions only')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        origin = joint.find('origin')
        xyz = tuple(map(float, origin.get('xyz', '0 0 0').split())) if origin is not None else (0., 0., 0.)
        rpy = tuple(map(float, origin.get('rpy', '0 0 0').split())) if origin is not None else (0., 0., 0.)
        if rpy != (0., 0., 0.):
            raise ValueError('Nonzero fixed rotation requires quaternion expectation extension')
        expected[child] = (parent, xyz)
    return expected


def test_one(share, model, logdir):
    expected = expected_transforms(share/'urdf'/f'{model}.urdf')
    observed = {}
    node = rclpy.create_node('visual_transfer_tf_probe')
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(TFMessage, '/tf_static',
        lambda msg: observed.update({tf.child_frame_id: tf for tf in msg.transforms}), qos)
    proc = None
    try:
        with (logdir/f'{model}_robot_state_publisher.log').open('w') as output:
            proc = subprocess.Popen(['ros2', 'launch', 'usv_rov_description',
                'display.launch.py', f'model:={model}', 'rviz:=false'],
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+15
            while time.monotonic()<deadline and not set(expected).issubset(observed):
                if proc.poll() is not None:
                    raise RuntimeError(f'ROS launch exited early: {proc.returncode}')
                rclpy.spin_once(node, timeout_sec=.2)
            missing = sorted(set(expected)-set(observed))
            if missing:
                raise AssertionError(f'Missing static transforms: {missing}')
            for child, (parent, xyz) in expected.items():
                tf = observed[child]
                actual = (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z)
                if tf.header.frame_id != parent or any(abs(a-b)>1e-8 for a,b in zip(actual, xyz)):
                    raise AssertionError(f'Transform mismatch for {child}: {actual} != {xyz}')
                q=tf.transform.rotation
                if any(abs(v)>1e-8 for v in (q.x,q.y,q.z)) or abs(abs(q.w)-1)>1e-8:
                    raise AssertionError(f'Rotation mismatch for {child}')
            return {'model':model, 'expected_transforms':len(expected), 'received_transforms':len(expected), 'passed':True}
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try: proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('package_share',type=Path)
    ap.add_argument('--log-dir',type=Path,default=Path('runtime_logs'))
    ap.add_argument('--model',action='append')
    args=ap.parse_args()
    args.log_dir.mkdir(parents=True,exist_ok=True)
    models=args.model or [p.stem for p in sorted((args.package_share/'urdf').glob('*.urdf'))]
    results=[]
    for index, model in enumerate(models):
        # Independent context/domain prevents stale transient-local samples from
        # previous models satisfying the next model's check.
        os.environ['ROS_DOMAIN_ID']=str(71+index)
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
        rclpy.init()
        try: results.append(test_one(args.package_share,model,args.log_dir))
        finally: rclpy.shutdown()
    output={'passed':all(r['passed'] for r in results),'test':'actual ROS launch and TF publication','models':results}
    (args.log_dir/'static_tf_results.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))


if __name__=='__main__':
    main()
