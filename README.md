# Catchrobo Battle Contest 2021 Firmware

# Environment
- ubuntu 18
- ros melodic

# How to use
## Display robot in Rviz
Without field:
```
roslaunch catarm_description catarm_display.launch 
```
With field (blue/red):
```
roslaunch catarm_description catarm_display.launch field:=blue
```