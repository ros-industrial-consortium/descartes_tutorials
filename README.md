# Descartes Tutorials

For a full description of the tutorials please see [the wiki](http://wiki.ros.org/descartes/Tutorials).

## Installation

Install the following:

```
sudo apt-get install ros-kinetic-moveit-simple-controller-manager
```

Clone the following repos:

```
git clone https://github.com/ros-industrial-consortium/descartes_tutorials.git
git clone https://github.com/ros-industrial/abb.git
git clone https://github.com/Jmeyer1292/opw_kinematics.git
git clone https://github.com/Jmeyer1292/descartes_opw_model.git
```

Install dependencies:

```
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Then build.

## Tutorial 1
Very basic tutorial. Moves a robot along a straight line by specifying where to
put the tool.
```
roslaunch descartes_tutorials setup.launch
```
```
rosrun descartes_tutorials tutorial1
```

## Tutorial 2
A more complex tutorial. The robot is holding a puzzle piece and moves the perimeter around a fixed grinder in the environment.
```
roslaunch tutorial2_moveit_config planning_execution.launch
```

```
rosrun descartes_tutorials tutorial2
```

## Tutorial 3 (Tutorial 2 w/ OPW Kinematics)
This tutorial shares 95% of its code with tutorial2 and solves the same problem, just using the
[opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics) kinematics solver package.

We re-use the workspace from tutorial 2:
```
roslaunch tutorial2_moveit_config planning_execution.launch
```

```
rosrun descartes_tutorials tutorial3
```


