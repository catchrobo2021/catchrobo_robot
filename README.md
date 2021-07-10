### Environment
- ubuntu 18
- ros melodic

### How to use
display robot in rviz without field:
```
roslaunch catarm_description catarm_display.launch 
```
display robot in rviz with blue side field:
```
roslaunch catarm_description catarm_display.launch field:=blue
```
display robot in rviz with red side field:
```
roslaunch catarm_description catarm_display.launch field:=red
```