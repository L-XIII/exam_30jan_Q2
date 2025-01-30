# TurtleSim Distance Control Project ( assignment1_rt )

Welcome to the TurtleSim Distance Control Project! This project involves controlling and monitoring two turtles in the TurtleSim simulation environment using ROS (Robot Operating System). The project consists of two main nodes:

1. `Distance`: This node keeps an eye on the distance between two turtles and stops them if they get too close to each other or to the simulation boundaries.
2. `UI`: This node provides a user interface for manually controlling the turtles.

## Prerequisites

Before you get started, make sure you have the following installed on your system:

- ROS (Robot Operating System)
- The `turtlesim` package

## Installation

1. **Create a ROS package:**

    First, navigate to your catkin workspace and create a new package:

    ```sh
    cd ~/catkin_ws/src
    catkin_create_pkg assignment1_rt rospy std_msgs geometry_msgs turtlesim
    ```

2. **Add the source files:**

    Copy the provided `Distance.cpp` and `UI.cpp` files into the `src` directory of your new package:

    ```sh
    cp /path/to/Distance.cpp ~/catkin_ws/src/assignment1_rt/src/
    cp /path/to/UI.cpp ~/catkin_ws/src/assignment1_rt/src/
    ```

3. **Update your CMakeLists.txt:**

    Open your `CMakeLists.txt` file and add the following lines to build the nodes:

    ```cmake
    ## Add the following lines in your CMakeLists.txt

    add_executable(Distance src/Distance.cpp)
    target_link_libraries(Distance ${catkin_LIBRARIES})
    add_dependencies(Distance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    add_executable(UI src/UI.cpp)
    target_link_libraries(UI ${catkin_LIBRARIES})
    add_dependencies(UI ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    ```

4. **Build the package:**

    Now, navigate back to your catkin workspace root and build your package:

    ```sh
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## Usage

1. **Start the TurtleSim simulation:**

    Launch the TurtleSim node:

    ```sh
    rosrun turtlesim assignment1_rt
    ```

2. **Run the Distance Node:**

    Start the node that monitors the distance between the turtles:

    ```sh
    rosrun assignmet1_rt Distance
    ```

3. **Run the UI Node:**

    Start the user interface node for manually controlling the turtles:

    ```sh
    rosrun assignment1_rt UI
    ```

## Nodes

### Distance Node (`Distance`)

This node monitors the distance between two turtles and ensures they don't get too close to each other or the simulation boundaries.

**Subscribed Topics:**
- `/turtle1/pose` (turtlesim/Pose): Position of the first turtle.
- `/turtle2/pose` (turtlesim/Pose): Position of the second turtle.

**Published Topics:**
- `/turtles/distance` (std_msgs/Float32): Distance between the two turtles.
- `/turtle1/cmd_vel` (geometry_msgs/Twist): Velocity commands for the first turtle.
- `/turtle2/cmd_vel` (geometry_msgs/Twist): Velocity commands for the second turtle.

### UI Node (`UI`)

This node allows you to manually control the turtles using simple command inputs.

**Services:**
- `/spawn` (turtlesim/Spawn): Service to spawn the second turtle at a specified location.

**Published Topics:**
- `/turtle1/cmd_vel` (geometry_msgs/Twist): Velocity commands for the first turtle.
- `/turtle2/cmd_vel` (geometry_msgs/Twist): Velocity commands for the second turtle.

