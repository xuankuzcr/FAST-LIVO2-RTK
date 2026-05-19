# FAST-LIVO2-RTK

FAST-LIVO2-RTK extends FAST-LIVO2 with RTK/GNSS-constrained global optimization for long-range LiDAR-inertial-visual mapping. It reduces accumulated drift by fusing GNSS observations in a factor graph backend and produces globally consistent trajectories and point cloud maps.

## Highlights

- RTK/GNSS post-processing constraints for long-trajectory drift correction.
- Factor graph-based global optimization built on GTSAM.
- Globally aligned point cloud map output before and after optimization.
- Hardware platform with LiDAR, camera, GNSS receiver, mini PC, and STM32 synchronization controller.
- Example RTK rosbag and reproducible command-line workflow.

## Results

The following section is reserved for before/after optimization comparisons. Place the result images under `docs/images/` and update the paths if needed.

| Before Optimization | After Optimization |
| --- | --- |
| <img src="docs/images/before_optimization.png" width="420" alt="Point cloud map before RTK optimization"> | <img src="docs/images/after_optimization.png" width="420" alt="Point cloud map after RTK optimization"> |

Optional trajectory comparison:

| FAST-LIVO2 Trajectory | FAST-LIVO2-RTK Trajectory |
| --- | --- |
| <img src="docs/images/trajectory_before.png" width="420" alt="Trajectory before RTK optimization"> | <img src="docs/images/trajectory_after.png" width="420" alt="Trajectory after RTK optimization"> |

## Hardware Platform

The table below summarizes the main devices used by the platform. Add detailed parameters, interfaces, and synchronization methods according to the actual data collection setup.

| Device | Image | Model Description |
| --- | --- | --- |
| LiDAR | <img src="docs/images/LiDAR.jpg" width="180" alt="Livox Mid-360 LiDAR"> | Model: Livox Mid-360<br>Key parameters: scan rate, field of view, timestamp source, LiDAR-IMU synchronization |
| Camera | <img src="docs/images/camera.jpg" width="180" alt="MV-CB016-10GC-S-W camera"> | Model: MV-CB016-10GC-S-W<br>Key parameters: resolution, frame rate, exposure mode, external trigger configuration |
| GNSS Receiver | <img src="docs/images/ublox.jpg" width="180" alt="u-blox ZED-F9P GNSS receiver"> | Model: u-blox ZED-F9P<br>Key parameters: frequency bands, RTK mode, PPS output, serial output, antenna configuration |
| Computing Unit | <img src="docs/images/n100.jpg" width="180" alt="N100 mini PC"> | Model: N100 mini PC<br>Key parameters: CPU, memory, storage, network interfaces, ROS runtime environment |
| Synchronization Controller | <img src="docs/images/stm32.jpg" width="180" alt="STM32 synchronization controller"> | Model: STM32<br>Key parameters: trigger signal generation, PPS input, GPIO timing, serial communication |

## Time Synchronization

The synchronization diagram shows the timing relationship between the LiDAR, camera, GNSS receiver, PPS/trigger signals, STM32 timing control, and ROS timestamps.

<div align="center">
  <img src="docs/images/sync.jpg" width="850" alt="Time synchronization diagram">
</div>

## Installation

### System Dependencies

#### Ubuntu and ROS

Ubuntu 18.04~20.04. See [ROS Installation](http://wiki.ros.org/ROS/Installation).

#### PCL, Eigen, and OpenCV

PCL>=1.8. See [PCL Installation](https://pointclouds.org/).

Eigen>=3.3.4. See [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

OpenCV>=4.2. See [OpenCV Installation](http://opencv.org/).

#### Sophus

Install the non-templated/double-only version of Sophus.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

#### Vikit

Vikit contains camera models and the math/interpolation functions required by this project. Vikit is a catkin project, so download it into the source folder of your catkin workspace.

```bash
# Different from the one used in fast-livo1
cd catkin_ws/src
git clone https://github.com/xuankuzcr/rpg_vikit.git
```

### RTK Dependencies

The RTK branch also depends on the GNSS ROS message package used by the u-blox/GVINS toolchain. Put it in the same catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git
```

GTSAM is used for factor graph-based post-processing optimization.

```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j$(nproc)
sudo make install
```

GeographicLib is used for converting geographic coordinates to local Cartesian coordinates.

```bash
sudo apt-get install libgeographic-dev ros-${ROS_DISTRO}-eigen-conversions
```

On Ubuntu 20.04, `libgeographic-dev` installs its CMake find module under `/usr/share/cmake/geographiclib`, which this repository's `CMakeLists.txt` adds to `CMAKE_MODULE_PATH`.

## Quick Start

Download the provided RTK test rosbag file: [RTK-extension-Dataset](https://drive.google.com/drive/folders/1fsUMNn0qgZ816zNcM7TCWYPf4QH1_1WO?usp=drive_link).

1. Launch the system and load the UAV/AGV configuration file:

```bash
roslaunch fast_livo AGV.launch
```

2. Play the rosbag. Once the sequence is finished, press `Enter` in the terminal running the launch file to trigger the backend optimizer.

```bash
rosbag play AGV-LVGO-01-s55.bag -s 55 --duration 110
```

3. Compare the generated global point cloud maps before and after optimization:

```bash
pcl_viewer -multiview 1 src/FAST-LIVO2/output/global_pcd/after_optimization.pcd src/FAST-LIVO2/output/global_pcd/before_optimization.pcd
```

Use absolute paths if your workspace layout differs from the command above.


## Dataset

- RTK test rosbag: [RTK-extension-Dataset](https://drive.google.com/drive/folders/1fsUMNn0qgZ816zNcM7TCWYPf4QH1_1WO?usp=drive_link)

## Acknowledgements

This repository is built on top of FAST-LIVO2 and uses several open-source libraries and packages, including GTSAM, GeographicLib, and `gnss_comm`.
