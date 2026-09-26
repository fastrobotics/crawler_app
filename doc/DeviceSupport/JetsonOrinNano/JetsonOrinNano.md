[Device Support](../DeviceSupport.md)

- [Jetson Orin Nano](#jetson-orin-nano)
- [Device Information](#device-information)
- [Guides](#guides)
- [Command Reference](#command-reference)
  - [OS Information](#os-information)
  - [Serial Number](#serial-number)
  - [Network Scan](#network-scan)
  - [Get Architecture](#get-architecture)
  - [Check Power Mode](#check-power-mode)

# Jetson Orin Nano


# Device Information 
| Hostname   | Model Information | Architecture | IP Address    | OS           | Serial Number |
| ---------- | ----------------- | ------------ | ------------- | ------------ | ------------- |
| GPUModule1 | Jetson Orin Nano  | `aarch64`    | 192.168.86.48 | Ubuntu 24.04 |               |

![](artifacts/NvidiaJetsonOrinNano.jpg)

# Guides
| Guide                                      |
| ------------------------------------------ |
| [Image Management](ImageManagement.md)     |
| [Build Instructions](BuildInstructions.md) |
| [Auto Launch](AutoLaunch.md)               |

# Command Reference


## OS Information
`lsb_release -a`

## Serial Number
`cat /proc/cpuinfo | grep Serial`

## Network Scan
`nmap -sP 192.168.86.0/23`

## Get Architecture
`uname -m`

## Check Power Mode
`sudo nvpmodel -q`