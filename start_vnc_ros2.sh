#!/usr/bin/env bash
set -e

# Start TigerVNC on :0 (maps to TCP 5900). No password for simplicity.
# For a password, run once inside the container: `vncpasswd`
# Start VNC manually with resolution control
if ! pgrep -x Xvnc >/dev/null 2>&1; then
  Xvnc :0 -geometry 1920x1080 -localhost no -SecurityTypes None &
fi

sleep 3
export DISPLAY=:0

# Start XFCE session
if ! pgrep -x xfce4-session >/dev/null 2>&1; then
  exec /usr/bin/startxfce4 &
fi
sleep 5
# Launch your app
exec ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
localization:=true world:=maze map:=maze/maze.yaml slam:=true nav:=true rviz:=true
