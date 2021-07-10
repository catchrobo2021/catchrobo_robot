# Catchrobo 2021 ROS

## Environment
- ubuntu 18
- ros melodic
## How to use
### Display robot in Rviz
Without field:
```
roslaunch catarm_description catarm_display.launch 
```
With field (blue/red):
```
roslaunch catarm_description catarm_display.launch field:=blue
```
### Execute motion planning with Moveit
demo:
```
roslaunch catarm_moveit demo.launch 
```
![demo](moveit_demo.png)