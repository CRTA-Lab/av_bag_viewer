# av_bag_viewer

ROS2 Python package for visualizing rosbag2 recordings from the Astro robot.
Includes two nodes:
- **bag_viewer** — plays back compressed RGB images and odometry with a timeline GUI
- **depth_viewer** — renders a live 3D point cloud from aligned depth + RGB frames

---

## 1. SSH into the Robot

Each Astro robot has a unique number. Replace `x` with your robot's number:

```bash
ssh astro@192.168.0.1x
```

Example for robot 3:
```bash
ssh astro@192.168.0.13
```

---

## 2. Check if the Docker Container is Running

```bash
docker ps
```

Look for a container named **`astro_hw`** in the output. If it is not listed, start it before continuing.

---

## 3. Enter the Docker Container

```bash
docker exec -it astro_hw /bin/bash
```

You are now inside the container with access to all ROS2 tools and the camera drivers.

---

## 4. Navigate to the Workspace Source Directory

```bash
cd /home/astro/astro_ws/src
```

---

## 5. Check for (or Create) the Bags Folder

```bash
ls
```

If a `bags` folder is not listed, create it:

```bash
mkdir bags
cd bags
```

If it already exists, just navigate into it:

```bash
cd bags
```

---

## 6. Record a ROS2 Bag

Record compressed RGB, aligned depth, and odometry from the RealSense D435:

```bash
ros2 bag record \
  /camera/camera/color/image_raw/compressed \
  /camera/camera/aligned_depth_to_color/image_raw/compressedDepth \
  /odom
```

A timestamped folder (e.g. `rosbag2_2026_03_24-14_08_28`) will be created in the current directory.
Press **Ctrl+C** to stop recording.

---

## 7. Drive the Robot to Collect Data

While recording is running, drive the robot around the environment using the joystick or teleoperation node to capture a variety of viewpoints and depth information.

---

## 8. Download the Bag to Your Machine

The `/home/astro/astro_ws/src/bags` directory inside the Docker container is shared with the Jetson host filesystem at the same path.
You can download the recorded bag from the **Jetson** (not from inside Docker) using `scp`:

```bash
scp -r astro@192.168.0.1x:/home/astro/astro_ws/src/bags/<bag_folder_name> ~/Desktop/
```

Example:
```bash
scp -r astro@192.168.0.13:/home/astro/astro_ws/src/bags/rosbag2_2026_03_24-14_08_28 ~/Desktop/
```

---

## 9. Visualize the Bag

### Build and source the package

```bash
cd ~/astro_ws
colcon build --packages-select av_bag_viewer
source install/setup.bash
```

### RGB image + odometry viewer

Plays back the compressed RGB camera stream and draws the robot's odometry path:

```bash
ros2 run av_bag_viewer bag_viewer /path/to/bag_folder
```

Example:
```bash
ros2 run av_bag_viewer bag_viewer ~/Desktop/rosbag2_2026_03_24-13_11_16
```

### 3D depth point cloud viewer

Reconstructs a coloured 3D point cloud from the aligned depth and RGB frames:

```bash
ros2 run av_bag_viewer depth_viewer /path/to/bag_folder
```

Example:
```bash
ros2 run av_bag_viewer depth_viewer ~/Desktop/rosbag2_2026_03_24-14_08_28
```

Controls in the depth viewer:
| Action | Control |
|---|---|
| Rotate view | Left mouse drag |
| Pan view | Middle mouse drag |
| Zoom | Scroll wheel |
| Play / Pause | Play button |
| Scrub frames | Timeline slider |
| Change playback speed | Speed dropdown |
| Change max depth | Max depth dropdown |
