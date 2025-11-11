#!/usr/bin/env bash
set -e

cd ../..
docker build -t turtlebot4:slam -f Dockerfile.slam_vnc .