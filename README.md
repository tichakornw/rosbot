# cc01_autonomous_robots

### SETUP

The ROS-free component contract layer lives in `autonomy-sim/autonomy_contracts`.
Plain C++ planning, control, and perception libraries should implement those
contracts and register components by name. `autonomy-sim/autonomy_ros2_wrapper`
keeps ROS 2/Nav2 integration internal and adapts the selected components from
YAML.

Default example components live in `autonomy-sim/external_components/reference_components`.
Add external GitHub links in the labeled Planning, Control, and Perception sections of
`autonomy-sim/external_components/component_manifest.cmake`.

### Component Links

GitHub READMEs cannot save editable text boxes into the repository, so the real
editable boxes live in `autonomy-sim/external_components/component_manifest.cmake`.

| Component area | Paste the external GitHub link in |
| --- | --- |
| Planning | `PLANNING COMPONENT LINK` |
| Control | `CONTROL COMPONENT LINK` |
| Perception | `PERCEPTION COMPONENT LINK` |

Each linked repo must expose a CMake library target and register one or more
components with `REGISTER_PLANNER_COMPONENT`, `REGISTER_CONTROLLER_COMPONENT`,
or `REGISTER_PERCEPTION_COMPONENT`.

### SIMULATION

The ros-autonomy sim contains the codebase for the gazebo simulation of the robot.

To start simulation, navigate to `ros-autonomy-sim` and run `just start-gazebo-sim`

> [!NOTE]
> [just](https://github.com/casey/just) is used to simplify execution 
>
> Install it with:
>
> ```bash
> curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --to /usr/bin
> ```

### Versions
- ROS 2 - Humble (Through the docker image `husarion/rosbot:humble-0.13.1-20240201`)
- Gazebo Simulation - Version 6.15.0 (Utilized through docker image `husarion/rosbot-gazebo:humble-0.13.0-20240115`)
- RViz version 11.2.6 (ROS 2) (Through docker image `husarion/rviz2:humble-11.2.6-20230809`)
- Base docker image for custom Navigation container - `husarion/navigation2:humble-1.1.12-20240123`
