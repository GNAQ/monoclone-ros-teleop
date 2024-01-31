# Monoclone-ROS teleop package

A ROS Noetic package for DRL robotics engineering. Powered by **Kinova Kortex**, it aims to operate a real or simulated Kinova Gen3 7-DoF robotic arm through the Python ZeroRPC-based interaction API. The arm is simulated using the Gazebo Classic 11 simulator and Moveit.

----

## Attention: existing performance issues

High-frequency (non-streaming) ZeroRPC calls may significantly degrade performance. The maximum FPS could only reach about 12 with a simulated camera operating at 640x480 RGBDepth and approximately 30 with 256x256.

## Quickstart

1. Under **Ubuntu 20.04** (corresponding to the ROS version), install ROS Noetic, Gazebo Classic 11, and (optionally) Docker.
2. Clone or download this repo, then arrange its contents as:
```plain
monoclone_ros
├─assets
│  ├─peg_ins
│  │  └─...
│  └─simcam
│      ├─materials
│      │  └─textures
│      └─meshes
├─build
│  └─...
├─devel
│  └─...
├─scripts
└─src
    └─monoclone_ros_teleop
        ├─launch
        ├─src
        │  └─teleop
        │     └─task
        └─test
```
3. Run the scripts in the terminal, with working directory = `*/monoclone_ros/`. Typically, you should start by running `build.sh`, followed by launching Gazebo using `run_gz.sh`, and then initiating teleop, simcam, and observer nodes.

> ### Attention: dealing with model files
> In the Gazebo simulator, a model consists of 'links' organized in the form of a rooted tree.
> The structure, along with the physical parameters of those links, is described using an `.sdf` file.
> And the 'geometric shape' of each link, or so-called 'mesh', is described in an `.stl` or `.dae` file.
> 
> **For those meshes to be imported correctly, you should check each model's `.sdf` file under path `*/monoclone/monoclone_ros/assets/` (or any other specified path), and make sure these paths correctly match where those mesh files are being placed.**
> Instead of storing models in the Gazebo library, we choose to manage them together as a monorepo, giving better efficiency when working inside a docker container.

4. Adjust runtime params in `src/teleop/config.py`, including server ports, simcam resolution, model assets directory, and task-specific params.

## Architecture

### The reason I use ZeroRPC && ROS compatibility

Since Hydra runtime env (and many other packages) conflicts with ROSpy RE, letting both development and evaluation for ML projects could be a painful experience  under the ROS environment. I have decided to separate the teleop module of _Monoclone_, my graduation design regarding robotic arm imitation learning, as a standalone part. 

These Python packages are well-organized under the ROSpy runtime environment, without any monkey patches.

Since ROS nodes are not allowed to perform parallel motion planning and many other tasks, I have decided to separate the functionalities into three nodes, namely `camera`, `teleop`, and `observer`.
### Teleop RPC API

Check and modify each node's functionality in corresponding Python files, e.g. `src/observer.py`, `src/main.py`, etc. APIs are implemented insider `src/teleop/*.py`. 

### ZeroRPC limitations

ZeroRPC is not a dedicated framework for inter-language API calls, especially between C++ and Python (which are the only two languages supported by ROS). Additionally, the performance of ZeroRPC is noticeably inferior to that of native Python calls. The sole reason I chose it was due to its coding efficiency.

## TODOs

- Integrate node launches into one launch file with launch params.
- Replace ZeroRPC to some high performance and Python-C++ friendly RPC framework (gRPC, etc.).
- Tidy the code, add more comments.


