# cc01_autonomous_robots

### SETUP

An example of how to set up multicost and compute is in planning-dev/examples/example_setup.cpp

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