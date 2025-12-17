# PowerShell script for bringing up object detection dev
$ErrorActionPreference = "Stop"
# Note: xhost + is not available on Windows, skip it or use alternative method
# xhost +

Set-Location ..\..

$IMAGE = "turtlebot4:semantic_slam-vnc"

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
    --name turtlebot4_object_detection_dev `
    -p 5901:5900 `
    -v "${currentDir}/object_detection:/workspace/object_detection" `
    -w /workspace/object_detection `
    "$IMAGE"

