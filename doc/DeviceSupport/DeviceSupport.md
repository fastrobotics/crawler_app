[README](../../README.md)

- [Device Support](#device-support)
- [Devices Supported](#devices-supported)
- [Config and Sync](#config-and-sync)
  - [1. Set the appropriate Scenario](#1-set-the-appropriate-scenario)
  - [2. Sync](#2-sync)

# Device Support

# Devices Supported
| Device                                     |
| ------------------------------------------ |
| [Raspberry Pi](RaspberriPi/RaspberryPi.md) |

# Config and Sync
## 1. Set the appropriate Scenario
```bash
cd ~/catkin_ws
ln src/crawler_app/scenarios/dev/ config
```

## 2. Sync
Run:
```bash
cd ~/catkin_ws
cd src/crawler_app/
./scripts/sync/syncSoftware.py -s remote -d ComputeModule1 -c ../../config/
```
