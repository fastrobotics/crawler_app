# Crawler App
__TOC__
- [Crawler App](#crawler-app)
  - [Overview](#overview)
- [Architecture](#architecture)
- [Systems](#systems)
- [Features](#features)
- [ToDo](#todo)
  - [This PR](#this-pr)
  - [Devices](#devices)
- [Setup](#setup)
- [Build](#build)
  - [Build and run Unit Tests](#build-and-run-unit-tests)
- [Execution](#execution)
- [Documentation](#documentation)

## Overview
This repo serves as a template for a typical development app that uses eROS content.

# Architecture
![](Legend.png)

# Systems
| Status | System |
| ------ | ------ |



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

| Device         | IP            | OS               |
| -------------- | ------------- | ---------------- |
| DevComputer2   | 192.168.86.21 | x86 Ubuntu 20.04 |
| ComputeModule1 | 192.168.86.40 | Raspbian 10      |

# Setup

Pre-Requisites:

- Ubuntu system running 20.04 LTS

1. Clone this repo using:
```bash
git clone --recurse-submodules https://github.com/fastrobotics/crawler_app.git
git submodule update --remote
```
2. Run the following:

```bash
cd <repo>
./scripts/setup_ide.sh
./scripts/setup_robot.sh
```

# Build
To build, run the following:
```bash
cd <workspace>
catkin_make
```

## Build and run Unit Tests
```bash
cd <workspace>
catkin_make
catkin_make tests
catkin_make run_tests
```

# Execution
To launch the main content, run the following (after following [Build](#build))
```bash
cd <workspace>
#source devel/setup.bash
#roslaunch robot_framework_ros robot.launch
```

# Documentation
