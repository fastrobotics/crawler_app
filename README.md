# Crawler App
__TOC__
- [Overview](#overview)
- [Setup](#setup)
- [Features](#features)
- [Software Design](#software-design)

## Overview
This repo serves as a template for a typical development app that uses eROS content.

## [System Design](doc/SystemDesign/SystemDesign.md)

## Setup
### Scenario Management
To set the scenario, perform the following:
1. Remove any config folders under catkin_ws/src/:
```bash
rm -rf ~/catkin_ws/src/config/`
```

2. Determine which scenario shall be used.  Currently supported scenario's:
- `dev`

3. Create a symlink from this scenario to the config directory:
```bash
cd ~/catkin_ws/src/
ln ~/<location>/crawler_app/scenarios/<scenario>/ config
```

## [Features](doc/Features/Features.md)

## Software Design
![](doc/output/Legend.png)
![](doc/output/CrawlerAppPackageDiagram.png)