

---

# 🐢 Turtle TF Follow – Leader–Follower Simulation (ROS2 Humble)

A simple and powerful **leader–follower behavioral system** built using **ROS2 Humble**, **TF2**, and **Turtlesim**.
This project demonstrates how one robot (turtle2) can automatically follow another robot (turtle1) by using **TF transforms** and a **proportional controller**.

This project is perfect for beginners who want to learn:

* How to use **TF2 frames**
* How to broadcast and lookup transforms
* How to control a robot using **geometry_msgs/Twist**
* How leader–follower logic works
* How to build multi-robot behaviors
* How to scale this concept to **Gazebo mobile robots**

---

## 🚀 Features

* **Manual or joystick control** of turtle1 (leader)
* Real-time **TF broadcasting** of leader pose
* **Follower controller** that keeps turtle2 following turtle1
* Clean project structure with launch files
* Can be adapted to **Gazebo robots**, **multi-robot systems**, and **cleaning robot simulations**

---

## 📦 Package Structure

```
turtle_tf_follow/
├── src/
│   ├── leader_tf_broadcaster.cpp
│   ├── turtle2_tf_broadcaster.cpp
│   └── follower_controller.cpp
├── launch/
│   └── follow.launch.py
├── CMakeLists.txt
└── package.xml
```

---

## 🧠 How It Works

### 1. **Leader Control**

`turtle1` is controlled manually using:

```bash
ros2 run turtlesim turtle_teleop_key
```

### 2. **Leader TF Publisher**

`leader_tf_broadcaster` publishes a TF frame:

```
frame_id: world  
child_frame_id: turtle1  
transform: (x, y, theta)
```

### 3. **Follower Controller**

The follower node:

* Looks up the transform: `turtle1 → turtle2`
* Calculates:

  * Distance to leader
  * Heading angle toward leader
* Publishes velocity to `/turtle2/cmd_vel`
* Uses a simple **P-controller** to follow smoothly

---

## 💡 Concepts Demonstrated

| Concept                  | Description                            |
| ------------------------ | -------------------------------------- |
| **TF2**                  | Transform broadcaster & lookup buffer  |
| **Topics**               | `/turtle1/cmd_vel`, `/turtle2/cmd_vel` |
| **Subscriptions**        | Pose & transforms                      |
| **Control Theory**       | Proportional controller for following  |
| **Multi-Robot Behavior** | Leader–follower robotics architecture  |

---

## 🛠️ Installation

Clone your ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone this package:

```bash
git clone https://github.com/<your_username>/turtle_tf_follow.git
```

Install dependencies:

```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

Build the package:

```bash
colcon build
source install/setup.bash
```

---

## ▶️ Run the Project

Start turtlesim:

```bash
ros2 run turtlesim turtlesim_node
```

Move leader turtle manually:

```bash
ros2 run turtlesim turtle_teleop_key
```

Launch full leader–follower system:

```bash
ros2 launch turtle_tf_follow follow.launch.py
```

Now **turtle2 will automatically follow turtle1** using TF-based control 🎉

---

## 📈 How It Can Be Extended

This same leader–follower architecture can be used for:

* Gazebo cleaning robot simulation
* Multi-robot patrol systems
* Swarm robotics
* UAV/UGV cooperative groups
* Convoy robots in warehouses
* Human-following shopping cart robot
* Autonomous mobile robot (AMR) systems

Just replace Turtlesim with:

* A Gazebo differential drive robot
* Real robots (TurtleBot3, Jackal, custom robots)
* Navigation stack (Nav2)

---

## 📚 Learning Outcome

By completing this project you learn:

✔ How to write TF2 broadcasters
✔ How to lookup transforms
✔ How to calculate relative pose
✔ How to implement velocity controllers
✔ How multi-robot systems are coordinated
✔ How to structure ROS2 packages
✔ How to generalize this to **any robot model**

---

## 🤝 Contributing

Pull requests are welcome!
If you want improvements such as:

* PID controller
* Obstacle avoidance
* Gazebo version
* Multi-follower swarm
* RViz visualization

Feel free to open an issue.

---

## 📜 License

MIT License
Free to use for education, research, or commercial projects.

---

