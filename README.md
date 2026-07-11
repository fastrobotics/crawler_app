# Crawler App

- [Crawler App](#crawler-app)
  - [Overview](#overview)
- [Architecture](#architecture)
- [System Design](#system-design)
- [Features](#features)
- [ToDo](#todo)
  - [This PR](#this-pr)
  - [Devices](#devices)
- [Setup](#setup)
- [Build](#build)
  - [Other Devices](#other-devices)
  - [Host](#host)
- [Software Sync](#software-sync)
- [Execution](#execution)
- [Device Support](#device-support)


## Overview
This repo serves as a template for a typical development app that uses eROS content.

# Architecture
![](Legend.png)

# System Design
[System Design](doc/SystemDesign/SystemDesign.md)


# Features
| Status | Feature |
| ------ | ------- |


# ToDo
| Item |
| ---- |



## This PR
| Item |
| ---- |

## Devices

| Device       | IP            | OS               |
| ------------ | ------------- | ---------------- |
| DevComputer2 | 192.168.86.21 | x86 Ubuntu 20.04 |


# Setup

Pre-Requisites:

- Ubuntu system running 20.04 LTS

1. Clone this repo using:
```bash
git clone --recurse-submodules https://github.com/fastrobotics/crawler_app.git
cd crawler_app
git submodule update --remote
```
2. Run the following:

```bash
cd <repo>
./scripts/setup_ide.sh
./scripts/setup_robot.sh
pre-commit install
```

# Build
## Other Devices
For other Device Build information, see:
| Device       | Build Instructions                                                       |
| ------------ | ------------------------------------------------------------------------ |
| Raspberry Pi | [Build Instructions](doc/DeviceSupport/RaspberriPi/BuildInstructions.md) |

## Host
To build on the host, run the following:
```bash
cd <workspace>
catkin_make
catkin_make tests
catkin_make run_tests
```

# Software Sync
To sync software to other devices, run the following:

```bash
cd ~/catkin_ws/src/crawler_app
./scripts/sync/syncSoftware.py -s remote -d ComputeModule1 -c scenarios/dev/
```

Note that after this, you will need to build the content on that device.  See [Build-Other Devices](#other-devices).

# Execution
To launch the main content, run the following (after following [Build](#build))
```bash
cd <workspace>
source devel/setup.bash
roslaunch crawler_app SystemLaunch.launch
```

# Device Support
[Device Support](doc/DeviceSupport/DeviceSupport.md)
