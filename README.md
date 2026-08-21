# franka_pick_and_place

Autonomous execution of a pick and place task, consisting of lifting a red cube with a **Franka FR3** robotic arm and an **RGB-D camera** capable of detecting obstacles and the object to be moved. The perception system identifies cube, obstacle, and target using HSV segmentation (double threshold for red, single for blue and green), followed by morphological filtering to remove sensor noise. For each object, the 2D centroid is calculated using the image's spatial moments, and the corresponding 3D position is directly extracted from the organized point cloud, then transformed into the world frame using TF2. To avoid starting the task while objects are still settling physically after spawning in Gazebo, the estimated position is only considered valid after remaining stable (Δd < 3 mm) for 15 consecutive frames (~1.5 s at 10 Hz).

Using the camera, the robot identifies the poses of 3D objects through segmentation in the three channels followed by the calculation of the respective centroid. After that, a Cartesian trajectory and a grasp phase are planned, the cube is lifted and transported towards the goal while avoiding the obstacle (blue). - All of this is done with a single launch command.

For performing the task, the robot uses **Cartesian** trajectories except for the obstacle avoidance movement, which uses an **RRT** planning + a safety distance.

📄 A full technical report (design, evaluation, results) is available in [Ros_Project.pdf](https://github.com/user-attachments/files/31265374/Ros_Project.pdf)



-------------------------------------------------------------------------------
# 🎥 Demo videos
front view:



https://github.com/user-attachments/assets/57eb5b07-b223-4eed-95e6-ebdc5d07316f



above view:

https://github.com/user-attachments/assets/b15bcc96-acbc-4f19-9b66-431f17fcef0a



-------------------------------------------------------------------------------
# Main parts of the code
**movit_node.cpp** (C++): The file defines the control functions to run the full pick-and-place sequence with the robotic arm. It logs obstacles detected in MoveIt's collision scene to ensure safe trajectories and manages the gripper opening and closing through a ROS 2 action client. Finally, it switches between free kinematic planning (RRTConnect) for long-range moves and straight Cartesian movements for approaching, picking up, lifting, and accurately placing the cube.

**perception_node.hpp** (C++):The node grabs RGB images and point clouds from the camera to detect a cube, an obstacle, and a target based on their color. Using HSV segmentation and TF2 transformations, it converts the 2D image centroids into 3D coordinates in the world space. Finally, it publishes these positions and monitors their stability for about a second and a half before confirming they are ready to use.

**main_node.cpp** (C++): The program initializes a ROS 2 node for visual perception and sets up MoveIt 2 to handle the kinematic control of the Franka FR3 robotic arm. After a short stabilization wait, the system stays on standby until the perception detects a cube, a target, and an obstacle. Once the coordinates of these elements are acquired and saved, the pick-and-place routine starts, and at the end of the operation, nodes and threads are cleanly shut down.


----------------------------------------------------------------------------------
# Structure of the code

franka_pick_and_place/

├── config/

│   └── moveit_controllers.yaml     # Configurazione dei controller di traiettoria MoveIt 2
├── include/

│   └── myworkcell_core/
│       └── perception_node.hpp     # Classe PerceptionNode, struct Object3D, soglie HSV

├── launch/
│   └── workcell.launch.py          # Avvio di simulazione, percezione e controllo in un unico comando

├── src/
│   ├── main.cpp                    # Entry point: inizializzazione nodi, threading, avvio missione
│   ├── moveit_node.cpp             # Logica pick & place: MoveGroup, planning cartesiano, gripper
│   ├── perception_node.cpp         # Segmentazione HSV, estrazione da point cloud, trasformazioni TF2
│   ├── myworkcell_node.cpp         # Gestione delle chiamate ai servizi ROS 2 della workcell
│   └── vision_node.cpp             # Nodo di localizzazione (service server) esposto al task controller
├── srv/                            # Definizioni dei servizi ROS 2 personalizzati (.srv)
├── CMakeLists.txt                  # Regole di compilazione CMake
├── package.xml                     # Dipendenze e metadati del pacchetto ROS 2
└── LICENSE


# Run 
       
 RUN: ```ros2 launch myworkcell_core workcell.launch.py```

 # Build 
BUILD: ```colcon build --packages-select myworkcell_core --symlink-install
source ~/ros2_ws/install/setup.bash```

# Results
20 independent trials were performed to evaluate the pick-and-place pipeline. The system achieved an 80% overall success rate (16/20 trials). Failures were mainly due to unsuccessful grasping (15%) and one collision caused by an incorrect trajectory (5%).
The placement phase achieved 100% success whenever the robot reached it, demonstrating accurate and repeatable final positioning.
Overall, the results confirm good system reliability, while highlighting grasping and collision-free trajectory planning as the main areas for improvement.

# Dependencis 
ROS 2 Humble
<br>MoveIt 2
<br>Gazebo (Ignition)
<br>The Franka FR3 simulation base environment (```franka_description```, ```franka_gazebo_bringup```, ```franka_fr3_moveit_config, controllers```). These are not part of this repository; the two packages here depend on them but do not modify them.















