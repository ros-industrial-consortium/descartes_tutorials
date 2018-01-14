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

[![Descartes Tutorial 2](https://img.youtube.com/vi/9SsYagiFPAw/0.jpg)](https://www.youtube.com/watch?v=9SsYagiFPAw) 

A more complex tutorial. The robot is holding a puzzle piece and moves the perimeter around a fixed grinder in the environment.
```
roslaunch tutorial2_moveit_config planning_execution.launch
```

```
rosrun descartes_tutorials tutorial2
```
