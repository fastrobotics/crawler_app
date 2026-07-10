#!/bin/bash
# REPO Config
## General Config
export REPO_NAME="crawler_app"
export BUILD_TOOL="catkin" #cmake, ...

## Coverage Config
export LINE_COVERAGE_THRESHOLD=50
export BRANCH_COVERAGE_THRESHOLD=0 # Don't care about Branch Coverage.