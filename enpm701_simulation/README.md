# enpm701_simulation
The objective of this personal project is to make a ros2 python package for a Webots simulation recreating the "final challenge" for course ENPM701 Autonomous Robots.
## Final Challenge
This challenge involved building a physical robot controlled by a raspberry pi and programming it to autonomously retrieve 9 blocks in order of color: red, green, blue, repeat. The robot begins in the 2 ft x 2 ft "landing zone" and must deliver all blocks to the 4 ft x 4 ft "construction zone" without knocking over any of the blocks. 
## Dependencies
| Category | Dependency
|---|---|
| ROS2 Distribution | Humble |
| Simulation | `webots_ros2_driver`, `rclpy` |
| Visualization | `cv2`, `cv_bridge` |

## Instructions:
### Build and Run:
Assuming existing ros2 workspace in home directory:
```bash
cd ~/ros2_ws/src
git clone https://github.com/dzinobile/enpm701_simulation.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select enpm701_simulation
source install/setup.bash
ros2 launch enpm701_simulation robot_launch.py

```
In a separate terminal, use one of these commands to launch the desired node:
```bash
ros2 run enpm701_simulation teleop_node.py

ros2 run enpm701_simulation colorpicker_node.py

ros2 run enpm701_simulation boundingboxes_node.py

ros2 run enpm701_simulation boundingboxes_node.py

ros2 run enpm701_simulation basic_autonomy_node.py
```
## Nodes 
- **my_robot_driver** - Custom driver for skid steer robot with front-mounted gripper and 2 wheel-mounted encoders.
- **teleop_node** - Custom teleoperation node for moving robot and opening/closing gripper.
- **colorpicker_node** - Node to facilitate selecting HSV bounds for block detection. Masked image viewable on /colorpicker_image/Image topic.
- **boundingboxes_node** - Node for troubleshooting box detection code. Similar to colorpicker but displays camera image with bounding boxes. Image viewable on /boundingboxes_image/Image topic.
- **basic_autonomy_node** - Currently runs autonomous routine to retrieve 3 blocks in color order.
## Package Structure
```
├── enpm701_simulation
│   ├── basic_autonomy_node.py
│   ├── boundingboxes_node.py
│   ├── colorpicker_node.py
│   ├── __init__.py
│   ├── my_robot_driver.py
│   └── teleop_node.py
├── launch
│   └── robot_launch.py
├── LICENSE
├── package.xml
├── README.md
├── resource
│   ├── enpm701_simulation
│   └── my_robot.urdf
├── setup.cfg
├── setup.py
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── worlds
    ├── Arena.proto
    ├── Block.proto
    ├── CustomBot.proto
    ├── meshes
    │   ├── block.dae
    │   └── block.STL
    └── my_world.wbt

```
