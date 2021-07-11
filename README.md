# Catchrobo 2021 ROS

## Environment
- ubuntu 18
- ros melodic
## How to use
### Display robot in Rviz
Without field:
```
roslaunch catchrobo_description catchrobo_display.launch 
```
With field (blue/red):
```
roslaunch catchrobo_description catchrobo_display.launch field:=blue
```
### Execute motion planning with Moveit
demo:
```
roslaunch catchrobo_moveit demo.launch 
```
![demo](moveit_demo.png)