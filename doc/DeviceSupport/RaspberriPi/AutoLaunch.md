[Raspberry Pi](RaspberryPi.md)
- [Auto Launch](#auto-launch)
  - [Systemctl auto launch service](#systemctl-auto-launch-service)
  - [Auto Launch Script](#auto-launch-script)
- [Troubleshooting](#troubleshooting)

# Auto Launch
The Raspberry Pi auto launches content at boot.  This is performed by the following:
## Systemctl auto launch service
A service is created on the raspberry pi, see [robot_launch service](artifacts/robot_launch.service)

To install this service on the Raspberry Pi, perform the following:
1. Copy this file to `/etc/systemd/system/`
2. Reload the daemon: `sudo systemctl daemon-reload`
3. Ensure the service is enabled: `sudo systemctl enable robot_launch.service`
4. Reboot the robot  

All that this service will do is to run a user-space launch script, see [Auto Launch Script](#auto-launch-script).

## Auto Launch Script
An Auto Launch script is specified for a given scenario.  This is dictated by the Device Type.

# Troubleshooting
You can inspect the status of this launch by running: `journalctl -u robot_launch.service -f`
