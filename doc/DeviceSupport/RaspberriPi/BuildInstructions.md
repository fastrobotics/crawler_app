[Raspberry Pi](RaspberryPi.md)

# Build Instructions
## Pre-Requisites
1. Sync Software to the Raspberry Pi

## Build
SSH into the Raspberry Pi and run:
```bash
cd ~/catkin_ws/
catkin_make -j2 -DCATKIN_ENABLE_TESTING=0
```