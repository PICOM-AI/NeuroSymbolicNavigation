# PowerShell script to run semantic SLAM
$ErrorActionPreference = "Stop"

Set-Location ..\..

$IMAGE = "turtlebot4:semantic_slam-vnc"

# Check if GPU is available
$GPU_ARG = ""
if (Get-Command nvidia-smi -ErrorAction SilentlyContinue) {
    $nvidiaCheck = nvidia-smi 2>&1
    if ($LASTEXITCODE -eq 0) {
        $GPU_ARG = "--gpus all"
    }
}

docker run --rm -it `
    $GPU_ARG `
    --ipc host `
    --name turtlebot4_semantic_slam `
    "$IMAGE" `
    bash -lc 'exec ros2 launch semantic_slam semantic_slam.launch.py'

