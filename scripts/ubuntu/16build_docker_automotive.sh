#!/usr/bin/env bash
set -e

cd ../..
docker build -t turtlebot4:automotive -f Dockerfile.automotive .