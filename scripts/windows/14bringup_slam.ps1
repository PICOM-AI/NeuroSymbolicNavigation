# PowerShell script for bringing up SLAM
$ErrorActionPreference = "Stop"
# Note: xhost + is not available on Windows, skip it or use alternative method
# xhost +

Set-Location ..\..

$IMAGE = "turtlebot4:slam"

$currentDir = (Get-Location).Path
docker run --rm -it `
    --name turtlebot4_slam `
    -p 5900:5900 `
    -v "${currentDir}/automotive:/ws" `
    -w /ws `
    "$IMAGE"

