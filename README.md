# Angry Turtle 
### ros-turtlesim-controller

> First assignment for Robotics course @ USI 19/20.

#### Contributors

**Giorgia Adorni** - giorgia.adorni@usi.ch   

[GiorgiaAuroraAdorni](https://github.com/GiorgiaAuroraAdorni)

#### Brief

A simple node that controls a turtle in **turtlesim**:

- the turtle is able to write 'USI' 
- when any other turtle gets closer than 2 meters to it at any given time, the turtle becomes angry, stops writing and starts pursuing the offender. 
- the offensive turtles are managed in another thread: they are spawned in a random position and move randomly (it is possible to control this turtle using the `turtle_teleop_key` from the turtlesim package)
- the angry turtle does not aim directly towards its target, but instead tries to look ahead m meters in front of the offender to intercept it and then eliminate it.
- after the elimination of the offender, the angry turtle moves back to the initial position and restarts its writing behavior, ignoring anyother turtle on the way back. 
- when all the turtles have been killed, the angry turtle writes 'USI' for the last time.

#### Prerequisites

* Python 2.0 
* ROS
* Turtlesim

#### Installation and usage

```sh
$ git clone https://github.com/giorgiaAuroraAdorni/ros-turtlesim-controller
```
#### Usage

The execution can be started simply using the following command:

```bash
$ roslaunch usi_angry_turtle angry_turtle.launch
```

The file `angry_turtle.launch` is a launch file that contains the instruction to configure the two nodes required: the `turtlesim_nod` and the node `angry_turtle_controller.py` that contains the implemented controller.

Alternatively, it is possible to start the simulation as follows:

- in the first command line window type:

  ```
  $ roscore 
  ```

- in another terminal, to launch turtlesime, type:

  ```
  $ rosrun turtlesim turtlesim_node
  ```

- in the last window, to start the program, type:

  ```
  $ rosrun usi_angry_turtle angry_turtle_controller.py
  ```

Alternatively, it is possible to control a turtle using the `turtle_teleop_key`. In this case it this necessary to spawn another turtle and type in another window 

```
$ rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/turtleTarget/cmd_vel
```

where `turtle1` is the angry turtle and `turtleTarget` is the offender.