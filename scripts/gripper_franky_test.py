#!/usr/bin/env python3
"""
Simple Franky Gripper test utility.

This script attempts to connect to a gripper using the `franky` Python API
and perform a few basic operations to validate connectivity and behavior.

Usage examples:
  # interactive (prompts before sending motions)
  python3 scripts/gripper_franky_test.py --ip 10.90.90.1

  # non-interactive (auto-confirm)
  python3 scripts/gripper_franky_test.py --ip 10.90.90.1 --yes --async

Notes:
- Requires the `franky` package to be installed in the Python environment where
  this script runs. If `franky` is not available, the script will print an
  informative error and exit.
- Adjust default widths/speeds/force/eps according to your gripper model and
  safety requirements before running on real hardware.
"""

import argparse
import time
import sys

try:
    from franky import Gripper
except Exception as e:
    Gripper = None


def wait_for_width(g, target, timeout=5.0, poll=0.05):
    """Poll gripper width until close to target or timeout."""
    start = time.time()
    while time.time() - start < timeout:
        try:
            w = float(g.width)
        except Exception:
            # if property access fails, just break
            return None
        if abs(w - target) < 1e-3:
            return w
        time.sleep(poll)
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', required=True, help='Gripper IP or identifier for Gripper()')
    parser.add_argument('--yes', action='store_true', help='Auto-confirm actions (no prompt)')
    parser.add_argument('--async', dest='use_async', action='store_true', help='Use async move where available')
    parser.add_argument('--timeout', type=float, default=5.0, help='Timeout (s) to wait for motions')
    args = parser.parse_args()

    if Gripper is None:
        print('ERROR: franky library not found. Install the franky package in this environment.', file=sys.stderr)
        print('If you intend to test without the hardware, consider mocking franky.Gripper or run on the robot host.', file=sys.stderr)
        sys.exit(2)

    print(f'Connecting to Gripper at {args.ip} ...')
    try:
        g = Gripper(args.ip)
    except Exception as e:
        print(f'Failed to create Gripper({args.ip}): {e}', file=sys.stderr)
        sys.exit(1)

    # Query some basic properties if available
    try:
        cur_width = float(g.width)
        print(f'Connected. Current gripper width: {cur_width:.4f} m')
    except Exception:
        print('Connected but could not read .width property (will continue).')

    if not args.yes:
        ok = input('Proceed to run test sequence on real hardware? [y/N]: ').strip().lower()
        if ok != 'y':
            print('Aborting per user request.')
            return

    # Test sequence
    try:
        print('\n1) Test MOVE to width=0.03 m')
        if args.use_async and hasattr(g, 'move_async'):
            print('Using move_async')
            g.move_async(0.03, 0.05)
        else:
            g.move(0.03, 0.05)
        w = wait_for_width(g, 0.03, timeout=args.timeout)
        print(f'  -> reached {w} (or None if not observed)')

        print('\n2) Test OPEN (speed=0.05)')
        if hasattr(g, 'open'):
            g.open(0.05)
            w = wait_for_width(g, 0.08, timeout=args.timeout)  # expect wider open; adjust as appropriate
            print(f'  -> observed width {w}')
        else:
            print('  -> gripper.open() not available on this API')

        print('\n3) Test GRASP (width=0.02, speed=0.03, force=20, eps=0.005)')
        if hasattr(g, 'grasp'):
            g.grasp(0.02, 0.03, 20.0, 0.005)
            w = wait_for_width(g, 0.02, timeout=args.timeout)
            print(f'  -> observed width {w}')
        else:
            print('  -> gripper.grasp() not available on this API')

        print('\n4) Test STOP')
        if hasattr(g, 'stop'):
            g.stop()
            time.sleep(0.2)
            try:
                print(f'  -> width after stop: {float(g.width):.4f}')
            except Exception:
                print('  -> could not read width after stop')
        else:
            print('  -> gripper.stop() not available')

    except Exception as e:
        print(f'Test sequence error: {e}', file=sys.stderr)

    # Optionally close or cleanup
    try:
        if hasattr(g, 'close'):
            g.close()
        elif hasattr(g, '__exit__'):
            g.__exit__(None, None, None)
    except Exception:
        pass

    print('Test finished.')


if __name__ == '__main__':
    main()
