# Franka-FACTR Bridge

Real-time bridge program connecting Franka Emika Panda robot to the FACTR teleoperation system via ZMQ communication.

## Overview

This bridge provides bidirectional communication between the Franka robot and FACTR teleoperation system:

- **Command Reception**: Receives joint position commands from FACTR (500Hz)
- **State Publishing**: Publishes robot joint positions and velocities (100Hz)
- **Torque Publishing**: Publishes external joint torques for force feedback (100Hz)

## Requirements

### Hardware
- Franka Emika Panda robot
- Network connection to robot (default IP: `10.0.10.2`)

### Software Dependencies
- Python 3.8+
- `franky` library (Franka Python interface)
- `zmq` (pyzmq)
- `numpy`

### Installation

```bash
# Install dependencies
pip install pyzmq numpy

# Install franky (follow Franka documentation)
# Typically: pip install franky
```

## Configuration

Edit the configuration constants at the top of `franka_factr_bridge_franky.py`:

```python
ROBOT_IP = "10.0.10.2"                    # Franka robot IP address
CMD_SUB_ADDRESS = "tcp://127.0.0.1:2098"  # Command subscription address
STATE_PUB_ADDRESS = "tcp://127.0.0.1:3099" # State publication address
TORQUE_PUB_ADDRESS = "tcp://127.0.0.1:3087" # Torque publication address

RELATIVE_DYNAMICS_FACTOR = 0.2            # Motion dynamics (0.2-1.0)
STATE_PUB_FREQUENCY = 100.0               # State publishing frequency (Hz)
```

### Network Configuration

For **local execution** (FACTR and bridge on same machine):
- Use `127.0.0.1` for all ZMQ addresses
- Ensure FACTR's `global_configs.py` matches these addresses

For **remote execution**:
- Replace `127.0.0.1` with the actual IP address
- Ensure firewall allows connections on ports 2098, 3099, 3087

## Usage

### Prerequisites

1. **Enable FCI on Franka**:
   - Access Franka Desk web interface: `https://10.0.10.2/desk/`
   - Enable FCI (Franka Control Interface) mode
   - Ensure robot is unlocked and ready

2. **Verify Network Connectivity**:
   ```bash
   ping 10.0.10.2
   ```

### Running the Bridge

```bash
python3 franka_factr_bridge_franky.py
```

The bridge will:
1. Connect to the Franka robot
2. Initialize ZMQ communication
3. Move robot to initial position
4. Start publishing state/torque data
5. Enter control loop to receive and execute commands

### Stopping the Bridge

Press `Ctrl+C` to gracefully stop the bridge. The robot will stop and resources will be cleaned up.

## Communication Protocol

### Command Format (FACTR → Bridge)
- **Protocol**: ZMQ SUB socket
- **Address**: `tcp://127.0.0.1:2098`
- **Message**: 7 × `float64` (56 bytes) - joint position targets
- **Frequency**: Up to 500Hz

### State Format (Bridge → FACTR)
- **Protocol**: ZMQ PUB socket
- **Address**: `tcp://127.0.0.1:3099`
- **Message**: 14 × `float32` (56 bytes)
  - First 7 floats: joint positions (rad)
  - Next 7 floats: joint velocities (rad/s)
- **Frequency**: 100Hz

### Torque Format (Bridge → FACTR)
- **Protocol**: ZMQ PUB socket
- **Address**: `tcp://127.0.0.1:3087`
- **Message**: 7 × `float32` (28 bytes) - external joint torques (Nm)
- **Frequency**: 100Hz

## Architecture

```
┌─────────────┐         ZMQ          ┌──────────────┐
│   FACTR     │ ◄─── State/Torque ───│              │
│  Teleop     │                       │    Bridge    │
│             │ ──── Commands ───────►│              │
└─────────────┘                       └──────┬───────┘
                                             │
                                             │ FCI
                                             ▼
                                      ┌─────────────┐
                                      │   Franka    │
                                      │    Robot    │
                                      └─────────────┘
```

### Threading Model

- **Main Thread**: Control loop (receives commands, executes motions)
- **Publisher Thread**: State/torque publishing loop (100Hz)

## Safety Features

1. **Initial Position**: Robot moves to safe initial position before accepting commands
2. **Error Recovery**: Automatic error recovery on motion exceptions
3. **Graceful Shutdown**: Proper cleanup on SIGINT
4. **Message Conflation**: Only latest command is executed (prevents command queue buildup)

## Troubleshooting

### Connection Issues

**Problem**: `Robot connection failed`
- **Solution**: Verify robot IP address and network connectivity
- Check FCI is enabled in Franka Desk
- Ensure no other program is using FCI

**Problem**: `ZMQ setup failed`
- **Solution**: Check if ports are already in use
- Verify firewall settings
- Ensure correct IP addresses in configuration

### Communication Issues

**Problem**: FACTR not receiving state/torque data
- **Solution**: 
  - Verify bridge is running and publishing
  - Check ZMQ addresses match in both programs
  - Ensure bridge starts before FACTR (or wait 2+ seconds)

**Problem**: Robot not following commands
- **Solution**:
  - Check command subscription is working
  - Verify `RELATIVE_DYNAMICS_FACTOR` is appropriate
  - Check robot is not in error state

### Performance Issues

**Problem**: High latency
- **Solution**:
  - Increase `RELATIVE_DYNAMICS_FACTOR` (0.2 → 0.5 → 1.0)
  - Check network latency
  - Verify no CPU bottlenecks

## Logging

The bridge uses Python's `logging` module with INFO level by default. Logs include:
- Connection status
- Error messages
- Important state transitions

To change log level, modify:
```python
logging.basicConfig(level=logging.DEBUG)  # For verbose output
```

## License

See main project license.

## References

- [FACTR Teleop Documentation](../README.md)
- [Franka FCI Documentation](https://frankaemika.github.io/docs/)
- [ZMQ Documentation](https://zeromq.org/)

