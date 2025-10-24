# NeuroSymbolicNavigation

## 1. Preparation

### 1.1. Robot Setup

#### Option 1: Simulation

Clone and launch the simulated TurtleBot environment:

```bash
git clone https://github.com/HiveIntel/TurtleBotSim.git
cd TurtleBotSim
# Follow the repository instructions to build and start the simulation
```

#### Option 2: Physical Robot

Follow the official setup guide for a real TurtleBot:
[https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html)

---

### 1.2. Environment Setup

#### Build the Docker Image

```bash
bash build.sh
```

#### Launch the Container

```bash
bash run.sh
```

---

### Notes

* Ensure Docker and Docker Compose are installed and configured correctly.
* Use the same network or ROS master configuration for both the robot and the container.
* For troubleshooting or custom configurations, refer to each component’s documentation.
