# franka_pick_and_place
Autonomous execution of a pick and place task, consisting of lifting a red cube with a Franka FR3 robotic arm and an RGB-D camera capable of detecting obstacles and the object to be moved. The perception system identifies cube, obstacle, and target using HSV segmentation (double threshold for red, single for blue and green), followed by morphological filtering to remove sensor noise. For each object, the 2D centroid is calculated using the image's spatial moments, and the corresponding 3D position is directly extracted from the organized point cloud, then transformed into the world frame using TF2. To avoid starting the task while objects are still settling physically after spawning in Gazebo, the estimated position is only considered valid after remaining stable (Δd < 3 mm) for 15 consecutive frames (~1.5 s at 10 Hz).

Using the camera, the robot identifies the poses of 3D objects through segmentation in the three channels followed by the calculation of the respective centroid. After that, a Cartesian trajectory and a grasp phase are planned, the cube is lifted and transported towards the goal while avoiding the obstacle (blue). - All of this is done with a single launch command.

For performing the task, the robot uses Cartesian trajectories except for the obstacle avoidance movement, which uses an RRT planning + a safety distance.

📄 A full technical report (design, evaluation, results) is available in Ros_Project.pdf

# 🎥  Demo videos
front view:

 Registrazione.dello.schermo.2026-08-11.181445.mp4 
above view:

 WhatsApp.Video.2026-08-20.at.14.20.02.mp4 
# Main parts of the code
***movit_node.cpp*** (C++): The file defines the control functions to run the full pick-and-place sequence with the robotic arm. It logs obstacles detected in MoveIt's collision scene to ensure safe trajectories and manages the gripper opening and closing through a ROS 2 action client. Finally, it switches between free kinematic planning (RRTConnect) for long-range moves and straight Cartesian movements for approaching, picking up, lifting, and accurately placing the cube.

***perception_node.hpp*** (C++):The node grabs RGB images and point clouds from the camera to detect a cube, an obstacle, and a target based on their color. Using HSV segmentation and TF2 transformations, it converts the 2D image centroids into 3D coordinates in the world space. Finally, it publishes these positions and monitors their stability for about a second and a half before confirming they are ready to use.

***main.cpp*** (C++): System entry point. Initializes ROS 2 and MoveIt 2 components, starts the execution threads, waits for the perception node to confirm the stable position of the cube, obstacle, and target, and finally launches the nine-stage pick-and-place sequence. At the end of the operation, it also handles the clean shutdown of nodes and threads.

***myworkcell_node.cpp*** (C++): Node for managing calls to the ROS 2 services of the workcell. Acts as a bridge between the task controller (main.cpp) and the services exposed by the perception system, encapsulating localization requests (via vision_node) into a reusable service interface for the rest of the pipeline.

***vision_node.cpp (C++)***: Exposes a ROS 2 Service Server ('Localizer') that provides the poses of detected objects via TF2 transformations, making the stable 3D coordinates of the cube, obstacle, and target available to the task controller.

 # Structure of the code
```
franka_pick_and_place/
├── config/
│   └── moveit_controllers.yaml     # Configuring MoveIt 2 Trajectory Controllers
├── include/
│   └── myworkcell_core/
│       └── perception_node.hpp     # PerceptionNode class, Object3D struct, HSV thresholds
├── launch/
│   └── workcell.launch.py          # Simulation startup, perception, and control all in one command
├── src/
│   ├── main.cpp                    # Entry point: node initialization, threading, mission start
│   ├── moveit_node.cpp             # Logica pick & place: MoveGroup, Cartesian planning, gripper
│   ├── perception_node.cpp         # HSV segmentation, extraction from point cloud, TF2 transformations
│   ├── myworkcell_node.cpp         # Handling calls to the ROS 2 services of the workcell
│   └── vision_node.cpp             # Localization node (service server) exposed to the task controller
├── srv/                            # Definitions of custom ROS 2 services (.srv)
├── CMakeLists.txt                  # CMake Build Rules
├── package.xml                     # Dependencies and metadata of the ROS 2 package
└── LICENSE
```
# Run
RUN: ```ros2 launch myworkcell_core workcell.launch.py```

# Build
BUILD: ```colcon build --packages-select myworkcell_core --symlink-install source ~/ros2_ws/install/setup.bash```

# Results
20 independent trials were performed to evaluate the pick-and-place pipeline. The system achieved an 80% overall success rate (16/20 trials). Failures were mainly due to unsuccessful grasping (15%) and one collision caused by an incorrect trajectory (5%). The placement phase achieved 100% success whenever the robot reached it, demonstrating accurate and repeatable final positioning. Overall, the results confirm good system reliability, while highlighting grasping and collision-free trajectory planning as the main areas for improvement.

# Dependencis
ROS 2 Humble
MoveIt 2
Gazebo (Ignition)
The Franka FR3 simulation base environment (franka_description, franka_gazebo_bringup, franka_fr3_moveit_config, controllers). These are not part of this repository; the two packages here depend on them but do not modify them.
