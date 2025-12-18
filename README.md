
# NeuroSymbolicNavigation

## Overview

This repository provides scripts and setup instructions to run the **NeuroSymbolic Navigation** system using Docker. It includes both simulation and solver components for TurtleBot navigation.

---

## Prerequisites

Ensure that **Docker** is installed and properly configured on your system.
Refer to the official guide: [Install Docker](https://docs.docker.com/engine/install/)

---

## 0. Scripts

For quick setup and testing, use the provided bash scripts:

```bash
├── 1build_docker_x11.sh              # Build the Docker image
├── 2bringup_docker_x11.sh            # Launch the simulator in a container
├── 3execute_solver.sh                # Run the ASP solver (dry run without robot)
├── 4execute_solver_with_robot.sh     # Run the ASP solver with robot control
├── 5execute_solver_map2.sh           # Alternate example (similar to script 3)
├── 6execute_solver_map2_with_robot.sh# Alternate example (similar to script 4)
├── preview_asp_map.sh                # Preview ASP map
└── ...
```

---

## 1. Preparation: Robot Simulation Setup

Clone the repository and launch the simulated TurtleBot environment:

```bash
git clone https://github.com/PICOM-AI/NeuroSymbolicNavigation.git
```

### For Linux (Ubuntu)

```bash
cd scripts/ubuntu

# Build the Docker image
bash 1build_docker_x11.sh

# Launch the simulator in a Docker container
bash 2bringup_docker_x11.sh
```

A simulation window will appear. You can interact with it using your mouse:

* **Pan:** Click and drag
* **Tilt:** Hold the `Shift` key and drag
* **Zoom:** Mouse Scroll

---

### For macOS (ARM/Intel)

macOS does not natively support X11 forwarding from Docker. To work around this, a VNC server is created inside the container, and you can connect to it via a VNC viewer.

```bash
cd scripts/macos

# Build the Docker image
bash 1build_docker_vnc.sh

# Launch the simulator with VNC support
bash 2bringup_docker_vnc.sh
```

After running script 2, open a VNC viewer and connect to `localhost:5900` (no password).
A simulation window will appear inside the VNC viewer.
Interaction is the same as on Linux:

* **Pan:** Click and drag
* **Tilt:** Hold the `Shift` key and drag
* **Zoom:** Mouse Scroll

> **Note:** We recommend using a third-party VNC client such as [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/?lai_vid=b5ROml3nAIvny&lai_sr=10-14&lai_sl=l) instead of the default macOS viewer.


### For Windows (WSL2)
For windows, please setup Windows Subsystem for Linux here [How to install WSL](https://learn.microsoft.com/en-us/windows/wsl/setup/environment).

Then open WSL terminal and run:
```bash
cd scripts/ubuntu

# Build the Docker image
bash 1build_docker_x11.sh

# Launch the simulator in a Docker container
bash 2bringup_docker_x11.sh
```

A simulation window will appear. You can interact with it using your mouse:

* **Pan:** Click and drag
* **Tilt:** Hold the `Shift` key and drag
* **Zoom:** Mouse Scroll


---


## 2. Physical Robot (Optional)

To deploy on a real TurtleBot, follow the official setup guide:
[TurtleBot 4 User Manual – Basic Setup](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html)

---

