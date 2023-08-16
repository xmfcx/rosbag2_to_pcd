# `rosbag2_to_pcd`

The `rosbag2_to_pcd` package provides functionality to convert point cloud data from a ROS 2 rosbag file to PCD (Point
Cloud Data) files.
For each point cloud message in the specified topic, the package creates an individual PCD file.

## Installation

To install, follow the [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and then execute:

```bash

mkdir -p ~/rosbag2pcd_ws/src
cd ~/rosbag2pcd_ws/src
git clone https://github.com/xmfcx/rosbag2_to_pcd.git
cd ~/rosbag2pcd_ws

sudo apt update
rosdep init
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

colcon build --symlink-install  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Usage

### Parameters

The node accepts the following parameters:

- **`path_bag`**: The absolute path to the rosbag2 file that you want to convert.
    - Example: **`/home/mfc/bags/rosbag2_2023_08_15-18_42_55`** which contains:
        - `/home/mfc/bags/rosbag2_2023_08_15-18_42_55/metadata.yaml`
        - `/home/mfc/bags/rosbag2_2023_08_15-18_42_55/rosbag2_2023_08_15-18_42_55_0.mcap`
- **`topic_cloud`**: The topic that contains the point cloud data in the rosbag2 file.

By default, the parameters are sourced from the `rosbag2_to_pcd.param.yaml` file in the `config` directory.
Modify this file's default values or provide your own parameter file as needed.

### Running the Node

Execute the following commands:

```bash
source ~/rosbag2pcd_ws/install/setup.bash
ros2 launch rosbag2_to_pcd rosbag2_to_pcd.launch.xml
```

Once launched, the node will process the rosbag file specified by the `path_bag` parameter.
It will generate a PCD file for each point cloud message, storing them in a directory named `<rosbag-name>_pcds` adjacent to the `rosbag2` folder.

The resulting PCD files use the timestamp of the point cloud message for naming, e.g., `1610472435-452138256.pcd`.

### Tips

To visually check which point cloud corresponds to which timestamp:
- Launch Rviz2 to view the point cloud.
- In a separate terminal, run:
   - `ros2 topic echo /your_point_cloud_topic --field header.stamp `
- Play your bag file.
- Pause the bag file playing by pressing `space` key when needed.
- Match the point cloud message's timestamp with the corresponding PCD file name.
