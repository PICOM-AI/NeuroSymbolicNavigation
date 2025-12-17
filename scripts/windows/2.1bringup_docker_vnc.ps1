# PowerShell script for bringing up Docker VNC with ROS2
$ErrorActionPreference = "Stop"
Set-Location ..\..

$IMAGE = "turtlebot4:vnc"

# Check if GPU is available
$GPU_ARG = ""
if (Get-Command nvidia-smi -ErrorAction SilentlyContinue) {
    $nvidiaCheck = nvidia-smi 2>&1
    if ($LASTEXITCODE -eq 0) {
        $GPU_ARG = "--gpus all -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute"
    }
}

$currentDir = (Get-Location).Path
docker run --rm -it `
    $GPU_ARG `
    --name turtlebot4_sim `
    -p 5900:5900 `
    -v "${currentDir}/solver:/workspace/solver" `
    "$IMAGE" "/usr/local/bin/start_vnc_ros2.sh"

