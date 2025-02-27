# Crawler App
## Overview
This repo serves as a template for a typical development app that uses eROS content.

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