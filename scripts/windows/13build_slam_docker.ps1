# PowerShell script to build SLAM Docker
$ErrorActionPreference = "Stop"

Set-Location ..\..
docker build -t turtlebot4:slam -f Dockerfile.slam_vnc .

