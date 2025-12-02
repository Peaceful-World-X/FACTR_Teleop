#!/usr/bin/env python3
"""
Simple ZMQ test publisher that sends a sequence of gripper commands (move, grasp, open, stop)
for testing the bridge's gripper SUB socket.

Usage:
  python3 scripts/gripper_pub_test.py --zmq tcp://127.0.0.1:2108 --dtype float32

This script does not use ROS; it directly publishes ZMQ binary numpy payloads.
"""

import argparse
import time
import numpy as np
import zmq


def send_cmd(sock, arr: np.ndarray):
    sock.send(arr.tobytes())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--zmq', default='tcp://127.0.0.1:2108', help='ZMQ address (bridge gripper SUB)')
    parser.add_argument('--dtype', choices=['float32', 'float64'], default='float32')
    args = parser.parse_args()

    dtype = np.float32 if args.dtype == 'float32' else np.float64

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.LINGER, 100)
    print(f'Connecting to {args.zmq}')
    sock.connect(args.zmq)
    time.sleep(0.1)

    try:
        while True:
            # move -> cmd 0 [0, width, speed]
            arr = np.array([0, 0.03, 0.05], dtype=dtype)
            print('Sending move', arr)
            send_cmd(sock, arr)
            time.sleep(1.0)

            # grasp -> cmd 1 [1, width, speed, force, eps]
            arr = np.array([1, 0.02, 0.03, 20.0, 0.005], dtype=dtype)
            print('Sending grasp', arr)
            send_cmd(sock, arr)
            time.sleep(1.0)

            # open -> cmd 2 [2, speed]
            arr = np.array([2, 0.05], dtype=dtype)
            print('Sending open', arr)
            send_cmd(sock, arr)
            time.sleep(1.0)

            # stop -> cmd 3 [3]
            arr = np.array([3], dtype=dtype)
            print('Sending stop', arr)
            send_cmd(sock, arr)
            time.sleep(2.0)

    except KeyboardInterrupt:
        print('Exiting')
    finally:
        sock.close()
        ctx.term()


if __name__ == '__main__':
    main()
