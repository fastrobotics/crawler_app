[Raspberry Pi](RaspberryPi.md)

# Build Instructions
## Pre-Requisites
1. Sync Software to the Raspberry Pi using [Sync Instructions](../../../README.md#software-sync).

## Build
SSH into the Raspberry Pi and run:
```bash
cd ~/ros2_ws/
colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-clean-cache
# Or Use the alias:
colcon_build
```