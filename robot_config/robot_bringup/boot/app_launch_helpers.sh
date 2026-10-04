#!/bin/bash

APP_LAUNCH_LOG=/home/robot/var/log/output/app_launch.out

message() {
    mkdir -p "$(dirname "$APP_LAUNCH_LOG")"
    printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" | tee -a "$APP_LAUNCH_LOG"
}