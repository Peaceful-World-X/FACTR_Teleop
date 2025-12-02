#!/usr/bin/env python3
"""
桥接发布测试脚本（在 bridge 主机上运行）

作用：在本机 3099/3087 端口上开启两个 PUB socket，周期性发送合成的
状态（14 个 float32）和力矩（7 个 float32）消息，用于排查网络/端口/防火墙
是否允许 FACTR 主机接收 bridge 的发布。

使用：
  在 bridge 主机上运行：
    python3 scripts/bridge_pub_test.py --state-address tcp://*:3099 --torque-address tcp://*:3087

然后在 FACTR 主机上运行：
    python3 scripts/test_bridge_recv.py --state tcp://<bridge_ip>:3099 --torque tcp://<bridge_ip>:3087 --timeout 10

如果 FACTR 能收到消息，则说明 bridge 主机的端口/网络可达；如果仍然收不到，说明网络或路由问题。
"""

import time
import argparse
import zmq
import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Bridge PUB tester: publish synthetic state/torque on given addresses")
    parser.add_argument("--state-address", default="tcp://*:3099", help="State PUB bind address (默认 tcp://*:3099)")
    parser.add_argument("--torque-address", default="tcp://*:3087", help="Torque PUB bind address (默认 tcp://*:3087)")
    parser.add_argument("--freq", type=float, default=10.0, help="发布频率 Hz（默认 10Hz，诊断用）")
    args = parser.parse_args()

    ctx = zmq.Context()
    pub_state = ctx.socket(zmq.PUB)
    pub_torque = ctx.socket(zmq.PUB)

    pub_state.bind(args.state_address)
    pub_torque.bind(args.torque_address)

    print(f"[BridgeTester] Bound state PUB: {args.state_address}")
    print(f"[BridgeTester] Bound torque PUB: {args.torque_address}")

    # 等待一小会儿，让 SUB 端有机会连接（ZeroMQ PUB/SUB 需要时间完成握手）
    time.sleep(0.5)

    period = 1.0 / args.freq
    seq = 0
    try:
        while True:
            # 合成状态：14 个 float64（7 pos + 7 vel），与 FACTR 接收端的默认解析保持一致
            pos = np.linspace(0.1, 0.7, 7, dtype=np.float64) + (seq % 100) * 1e-6
            vel = np.zeros(7, dtype=np.float64)
            state_msg = np.concatenate([pos, vel]).astype(np.float64)

            # 合成力矩：7 个 float64
            torque_msg = (0.01 * np.ones(7, dtype=np.float64)) * (1.0 + (seq % 5))

            try:
                pub_state.send(state_msg.tobytes())
                pub_torque.send(torque_msg.tobytes())
            except Exception as e:
                print(f"[BridgeTester] send error: {e}")

            # 每 50 条打印一次统计，便于远程观察
            if seq % 50 == 0:
                print(f"[BridgeTester] sent seq={seq} state_bytes={state_msg.nbytes} torque_bytes={torque_msg.nbytes}")

            seq += 1
            time.sleep(period)
    except KeyboardInterrupt:
        print("[BridgeTester] Interrupted by user.")
    finally:
        pub_state.close()
        pub_torque.close()
        ctx.term()


if __name__ == '__main__':
    main()
