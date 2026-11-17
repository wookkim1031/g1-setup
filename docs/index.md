---
layout: default
---

These pages document how the tools around a **Unitree G1** humanoid robot are set up in a lab hall. Each
page covers one setup: what the tool is, what was done, the commands, and the things that went wrong.
They are written for someone who has not used the tools before.

The pages describe setup and tools only. Experiment results are not part of them.

## Pages

| Setup | What it covers | Status |
|---|---|---|
| [Basics](basics.html) | **Start here.** LiDAR and its technologies, IMU, odometry, localisation and TF, explained from the sensor up | Done |
| [Navigation](navigation.html) | LiDAR, odometry and mapping with DLIO, localisation, Nav2, and the split between robot and server | Mapping done; localisation and Nav2 in progress |
| Simulation | Isaac Lab, the whole-body tracker and how a check run works | Planned |
| [Digital twin](digital-twin.html) | From a LiDAR map to a scene in Isaac Lab and MuJoCo, and scanning one chair | Done, with limits listed on the page |
| Voice | Wake word, speech-to-text, language model and text-to-speech | Planned |

All scripts are in the repository: [bin/](https://github.com/wookkim1031/g1-setup/tree/main/bin).

## Words used on every page

| Word | Meaning |
|---|---|
| **ROS 2** | A set of libraries that lets many small programs (nodes) work together to run a robot. They exchange messages on named streams called topics. The release used here is Humble, on Ubuntu 22.04. |
| **DDS** | The network layer underneath ROS 2. The G1 also publishes its sensors directly on DDS, so they can be read without ROS. |
| **Domain ID** | A number that keeps groups of DDS programs apart on one network. The G1 uses domain 0. Our own tools use other numbers so they never mix with the robot's traffic. |
| **Docker** | Packs a program with everything it needs into a container, so it runs the same on every PC. |
| **Onboard computer** | The computer inside the robot. Anything that must keep working when the wireless link drops has to run there. |
