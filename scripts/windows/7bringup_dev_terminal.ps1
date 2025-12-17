# PowerShell script for bringing up dev terminal
$ErrorActionPreference = "Stop"
Set-Location ..\..

$IMAGE = "turtlebot4:vnc"
$CONTAINER_NAME = "turtlebot4_sim_dev"

# Check if GPU is available
$GPU_ARG = ""
if (Get-Command nvidia-smi -ErrorAction SilentlyContinue) {
    $nvidiaCheck = nvidia-smi 2>&1
    if ($LASTEXITCODE -eq 0) {
        $GPU_ARG = "--gpus all -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute"
    }
}

# Check if container exists
$containerExists = docker ps -a --format "{{.Names}}" | Select-String -Pattern "^${CONTAINER_NAME}$"
if ($containerExists) {
    Write-Host "Container ${CONTAINER_NAME} exists. Using docker exec..."
    
    # Check if container is running
    $containerRunning = docker ps --format "{{.Names}}" | Select-String -Pattern "^${CONTAINER_NAME}$"
    if ($containerRunning) {
        Write-Host "Container is running. Executing bash..."
        docker exec -it -w /workspace/solver "$CONTAINER_NAME" bash
    } else {
        Write-Host "Container is stopped. Starting container..."
        docker start "$CONTAINER_NAME"
        Write-Host "Executing bash..."
        docker exec -it -w /workspace/solver "$CONTAINER_NAME" bash -lc "pgrep -x Xvnc >/dev/null 2>&1 || Xvnc :0 -geometry 1920x1080 -localhost no -SecurityTypes None & sleep 3 && export DISPLAY=:0 && pgrep -x xfce4-session >/dev/null 2>&1 || exec /usr/bin/startxfce4 > /dev/null 2>&1 & bash"
    }
} else {
    Write-Host "Container ${CONTAINER_NAME} does not exist. Creating new container..."
    $currentDir = (Get-Location).Path
    docker run --rm -it `
        $GPU_ARG `
        --name "$CONTAINER_NAME" `
        -p 5900:5900 `
        -v "${currentDir}/solver:/workspace/solver" `
        -w /workspace/solver `
        "$IMAGE" `
        bash -lc "pgrep -x Xvnc >/dev/null 2>&1 || Xvnc :0 -geometry 1920x1080 -localhost no -SecurityTypes None & sleep 3 && export DISPLAY=:0 && pgrep -x xfce4-session >/dev/null 2>&1 || exec /usr/bin/startxfce4 > /dev/null 2>&1 & bash"
}

