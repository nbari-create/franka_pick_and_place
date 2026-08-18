Autonomous execution of a pick and place task, consisting of lifting a red cube with a **Franka FR3** robotic arm and an **RGB-D camera** capable of detecting obstacles and the object to be moved.

Using the camera, the robot identifies the poses of 3D objects through segmentation in the three channels followed by the calculation of the respective centroid. After that, a Cartesian trajectory and a grasp phase are planned, the cube is lifted and transported towards the goal while avoiding the obstacle (blue). - All of this is done with a single launch command.

For performing the task, the robot uses **Cartesian** trajectories except for the obstacle avoidance movement, which uses an **RRT** planning + a safety distance.

📄 A full technical report (design, evaluation, results) is available in xxxx

-------------------------------------------------------------------------------
# 🎥 Demo videos
