# fr3_motion
Controllers and utilities for moving Franka Research 3 robots using ROS2.

Usage
=====

After building and sourcing the workspace, bring up the system:

```bash
ros2 launch franka_bringup franka.launch.py robot_type:=fr3 \
       controllers_yaml:=src/fr3_motion/config/controllers.yaml
```

To use the `joint_impedance_with_ik_controller`, you must launch the move group.

```bash
ros2 launch fr3_motion move_group.launch.py
```

Then, load or unload the controller you wish to use.

```
ros2 control load_controller --set-state active joint_impedance_controller
ros2 control set_controller_state joint_impedance_controller inactive
ros2 control load_controller --set-state active joint_impedance_with_ik_controller
ros2 control set_controller_state joint_impedance_with_ik_controller inactive
```

The controllers subscribe to `/joint_state_setpoint` or `/ee_pose_setpoint`, 
which can be configured with parameters. Example nodes for generating 
setpoints are 

```bash
ros2 launch fr3_motion example_joint_impedance_commander
```

and

```bash
ros2 launch fr3_motion example_joint_impedance_with_ik_commander
```

Development Container
=====================
The packages in this workspace implement controllers and utilities for moving
FR3 robots from Franka using ROS. The controllers can be developed and deployed
using the container and compose file at the repository root. The container
described in the Dockerfile at the repository root is built automatically
using GitHub Actions and pushed to the GitHub Container Registry 
<https://ghcr.io/dimasad/fr3_motion>.

To start the container for development, run the following at the repository root.

```bash
docker compose up -d
```

Then, to open an interactive terminal on the container, run the following.
You may use this command to open multiple terminals, as typical in ROS
workflows. The repository root is mapped to `/workspace/` inside the
container and can be used for development.

```bash
docker compose exec dev bash
```

Once in the container, enter, build, and source the workspace.

```bash
cd /workspace/
colcon build --symlink-install
source install/setup.bash
```

After you are done, you may tear down the container with the command below,
which removes the container instance. All files outside `/workspace` and any
changes made to the container will be lost.

```bash
docker compose down
```
