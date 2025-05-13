# DeepSand - ROS 2 Package: `deploadblade`

`deploadblade` is a ROS 2-based software architecture for semi-autonomous surface grading. It enables communication between a teleoperation host and a client-side embedded control unit (ESP32) to command robot motion, blade actuation, and monitor sensory feedback like depth and load.

---

## 📦 Package Structure

The system is divided into two main parts:

### 🔹 Host Nodes (launched via `dlb-host.launch`)
- **`teleop_host`**: Publishes velocity and blade commands (`cmd_vel`, `blade_angle`).
- **`robotpose_publisher`**: Publishes estimated robot pose.
- **`depth_to_img`**: Subscribes to raw depth data and converts it for visualization.

### 🔹 Client Nodes (launched via `dlb-client.launch`)
- **`teleop_client`**: Subscribes to control commands and sends actuation signals to:
  - `ESP32-dev` (servo and load cell)
  - `Motor driver`
- **`depth_publisher_e`**: Publishes `depth_raw` data from an onboard depth sensor.

---

## 📡 ROS 2 Topics

| Topic Name       | Direction        | Type                | Description                      |
|------------------|------------------|---------------------|----------------------------------|
| `/cmd_vel`       | Host ➝ Client    | `geometry_msgs/Twist` | Velocity commands               |
| `/blade_angle`   | Host ➝ Client    | `std_msgs/Float32`  | Blade servo angle command       |
| `/robot_pose`    | Host ➝ Client    | `geometry_msgs/PoseStamped` | Robot pose data           |
| `/depth_raw`     | Client ➝ Host    | `sensor_msgs/Image` | Raw depth image                 |
| `/load_cell_data`| Client ➝ Host    | `std_msgs/Float32`  | Force data from blade's load cell |

---

## 🚀 Launch Instructions

### 1. Build
```bash
colcon build --packages-select deploadblade
source install/setup.bash
