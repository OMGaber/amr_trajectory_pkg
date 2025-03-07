### **README.md for `amr_trajectory_pkg`**

```markdown
# amr_trajectory_pkg

`amr_trajectory_pkg` is a ROS package designed to record, save, and visualize the trajectory of an autonomous mobile robot (AMR). It supports multiple file formats (CSV, JSON, YAML) for saving and reading trajectories and provides RViz integration for visualization.

---

## **Features**
- 📌 Record and save robot trajectories in **CSV, JSON, or YAML** formats.
- 📌 Read and publish saved trajectories using ROS topics.
- 📌 Visualize trajectories in **RViz** using markers.
- 📌 Supports time-stamped poses with position and orientation data.

```
## **Package Structure**

amr_trajectory_pkg/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── launch/                 # Launch files
│   ├── trajectory_saver.launch
│   ├── trajectory_reader.launch
├── rviz/                   # RViz config files
│   ├── trajectory_view.rviz
├── saved_trajectory/       # Saved trajectory files (CSV, JSON, YAML)
│   ├── trajectory.csv
│   ├── trajectory.json
│   ├── trajectory.yaml
├── src/                    # Source code
│   ├── trajectory_saver_node.cpp
│   ├── trajectory_reader_node.cpp
├── srv/                    # Service files
│   ├── SaveTrajectory.srv
├── include/                # Header files (if any)
└── README.md               # Documentation
```
---
```
## **Dependencies**
- ROS Noetic
- `yaml-cpp` for YAML parsing
- `nlohmann_json` for JSON parsing
- RViz for visualization

**Install dependencies:**
```bash
sudo apt-get install ros-noetic-yaml-cpp
sudo apt-get install nlohmann-json3-dev
```

---

## **Installation**
Clone this package into your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone <repository_link> amr_trajectory_pkg
```

Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## **Usage**

### **1. Saving Trajectories**
Run the `trajectory_saver_node` to record and save trajectories:
```bash
roslaunch amr_trajectory_pkg trajectory_saver.launch
```

Save the trajectory using a ROS service:
```bash
rosservice call /save_trajectory "{filename: '/root/race_ws/src/amr_trajectory_pkg/saved_trajectory/trajectory.csv', duration: 10.0}"
```

---

### **2. Reading and Publishing Trajectories**
Run the `trajectory_reader_node` to read and publish saved trajectories:
```bash
roslaunch amr_trajectory_pkg trajectory_reader.launch format:=csv
```

Supported formats:
```bash
roslaunch amr_trajectory_pkg trajectory_reader.launch format:=csv    # Default
roslaunch amr_trajectory_pkg trajectory_reader.launch format:=yaml   # YAML format
roslaunch amr_trajectory_pkg trajectory_reader.launch format:=json   # JSON format
```

---

### **3. Visualizing Trajectories in RViz**
Run RViz with a saved config:
```bash
rviz -d /root/race_ws/src/amr_trajectory_pkg/rviz/trajectory_view.rviz
```

---

## **Topics**

| Topic                                    | Type                                    | Description                                       |
|------------------------------------------|-----------------------------------------|---------------------------------------------------|
| `/trajectory_markers`                    | `visualization_msgs/MarkerArray`        | Marker array for RViz visualization.               |
| `/save_trajectory`                       | `amr_trajectory_pkg/SaveTrajectory`     | Service to save recorded trajectories.             |
| `/amcl_pose`                             | `geometry_msgs/PoseWithCovarianceStamped` | Pose topic for recording trajectories.            |

---

## **Service**

### `/save_trajectory`
**Type:** `amr_trajectory_pkg/SaveTrajectory`

**Request:**
- `string filename` - Path to save the trajectory file.
- `float32 duration` - Duration to record the trajectory.
- `string format` - File format: `csv`, `json`, or `yaml`.

**Response:**
- `bool success` - Success status.
- `string message` - Result message.

**Example:**
```bash
rosservice call /save_trajectory "{filename: '/root/race_ws/src/amr_trajectory_pkg/saved_trajectory/trajectory.yaml', duration: 10.0, format: 'yaml'}"
```

---

## **Customization**

### **Change File Path**
Modify the path in `trajectory_reader.launch`:
```xml
<param name="trajectory_file" value="/root/race_ws/src/amr_trajectory_pkg/saved_trajectory/trajectory.csv" />
```

---

## **Troubleshooting**

### **1. Missing Libraries Error**
If you get missing library errors, try:
```bash
sudo apt-get install ros-noetic-yaml-cpp nlohmann-json3-dev
```

### **2. RViz Not Displaying Trajectory**
- Check if `/trajectory_markers` topic is being published:
```bash
rostopic list | grep trajectory_markers
```
- Ensure correct frame ID in RViz (`map` or `odom`).
