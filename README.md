# Catchrobo 2021 ROS

## Environment
- ubuntu 18
- ros melodic

## Requirement
- moveit (see : https://moveit.ros.org/install/)
- sudo apt install ros-melodic-joint-state-publisher-gui 
- pip install pandas

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

### feature_control
- roslaunch catchrobo_manager bringup.launch
- rosrun catchrobo_manager game_manager.py




<!-- 
## IKFast  
- download following by http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html  (export MYROBOT_NAME=" catchrobo_description/robots/catchrobo")  
- display collada file:
```
$ openrave-robot.py catchrobo_description/robots/catchrobo.dae --info links

name              index parents          
-----------------------------------------
world             0                      
base/base_link    1     world            
base/link_tip     2     base/base_link   
arm/link0         3     base/link_tip    
arm/link1         4     arm/link0        
arm/link2         5     arm/link1        
arm/link3         6     arm/link2        
arm/link4         7     arm/link3        
arm/link5         8     arm/link4        
arm/link_tip      9     arm/link5        
gripper/base_link 10    arm/link_tip     
gripper/finger1   11    gripper/base_link
gripper/finger2   12    gripper/base_link
gripper/link_tip  13    gripper/base_link
-----------------------------------------
name              index parents          

```
- generate IK solver
```
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=catchrobo_description/robots/catchrobo.dae --iktype=transform6d --baselink="2" --eelink="8" --savefile="`pwd`/catchrobo_IKFast/ikfast61_arm0.cpp"
``` -->



<a href="http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html">moveit_commander: moveit_commander.move_group.MoveGroupCommander Class Reference</a><br><a href="http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#a4f87583115be2665c60332a756ddc67e">moveit_commander: moveit_commander.move_group.MoveGroupCommander Class Reference</a><br><a href="http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#a134b55e0efda49111dd9deb57657e180">moveit_commander: moveit_commander.move_group.MoveGroupCommander Class Reference</a><br><a href="http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#a16e46547320f3f2336a27d7e03c36dcf">moveit_commander: moveit_commander.move_group.MoveGroupCommander Class Reference</a><br><a href="https://search.yahoo.co.jp/search?p=moveit+jointstate&x=wrt&aq=-1&ai=d36510f2-a2eb-44a1-b2fd-4e2514683a17&ts=1872&ei=UTF-8&fr=mcafeess1">「moveit jointstate」の検索結果 - Yahoo!検索</a><br><a href="http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/PositionIKRequest.html">moveit_msgs/PositionIKRequest Documentation</a><br><a href="https://robo-marc.github.io/moveit_documents/moveit_commander.html">Python用ユーザライブラリ（moveit_commander）の仕様 — MoveIt! Documentation</a><br><a href="https://uenota.github.io/dronedoc/ja/moveit/drive_drone/drive_drone.html">MoveIt!を使ってGazeboモデルを動かす — Dronedoc 1.0.0 ドキュメント</a><br><a href="https://answers.ros.org/question/313637/openclose-end-effector-with-moveit-rviz/">Moveit Rvizを使用したオープン/クローズエンドエフェクター-ROSAnswers：オープンソースQ＆Aフォーラム</a><br><a href="http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html#the-entire-code">Pick and Place Tutorial — moveit_tutorials Kinetic documentation</a><br><a href="http://docs.ros.org/en/kinetic/api/moveit_msgs/html/msg/Grasp.html">moveit_msgs/Grasp Documentation</a><br><a href="https://github.com/ros-planning/moveit_grasps">ros-planning/moveit_grasps: Geometric grasping generator library for cuboids</a><br><a href="https://ros-planning.github.io/moveit_tutorials/doc/moveit_grasps/moveit_grasps_tutorial.html">MoveIt Grasps — moveit_tutorials Noetic documentation</a><br><a href="https://github.com/ros-planning/moveit/tree/master/moveit_ros/manipulation/move_group_pick_place_capability/src">moveit/moveit_ros/manipulation/move_group_pick_place_capability/src at master · ros-planning/moveit</a><br><a href="https://search.yahoo.co.jp/search?ei=UTF-8&fr=mcafeess1&p=moveit+pick+and+place">「moveit pick and place」の検索結果 - Yahoo!検索</a><br><Div Align="right" style="font-size:xx-small"><a href="http://romberg-iso8.blogspot.com/">powered by swordsmith</a></div>


<a href="https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/pick_place/src/pick_place_tutorial.cpp">moveit_tutorials/pick_place_tutorial.cpp at kinetic-devel · ros-planning/moveit_tutorials</a><br><a href="https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/pick_place/src/pick_place_tutorial.cpp#L137">moveit_tutorials/pick_place_tutorial.cpp at kinetic-devel · ros-planning/moveit_tutorials</a><br><a href="https://moveit.readthedocs.io/en/latest/doc/pr2_tutorials/planning/src/doc/planning_scene_ros_api_tutorial.html">ROS API Planning Scene Tutorial — moveit_tutorials Indigo documentation</a><br><a href="https://github.com/ros-planning/moveit/blob/master/moveit_commander/demos/pick.py">moveit/pick.py at master · ros-planning/moveit</a><br><a href="https://github.com/ros-planning/moveit/tree/master/moveit_commander/src/moveit_commander">moveit/moveit_commander/src/moveit_commander at master · ros-planning/moveit</a><br><a href="https://github.com/ros-planning/moveit/blob/4d1198e725fcd07b7e01a6f8e2411fb8c0e522b6/moveit_commander/src/moveit_commander/move_group.py#L697">moveit/move_group.py at 4d1198e725fcd07b7e01a6f8e2411fb8c0e522b6 · ros-planning/moveit</a><br><a href="https://search.yahoo.co.jp/search?ei=UTF-8&fr=mcafeess1&p=moveit+endeffector">「moveit endeffector」の検索結果 - Yahoo!検索</a><br><a href="https://qiita.com/srs/items/5501a5d5059e4596f8f0">ROS講座104 moveitでオリジナルのアームを使う - Qiita</a><br><a href="https://github.com/nazir-ust/ur10_cm/blob/master/launch/dual_arm_execution.launch">Page not found · GitHub</a><br><a href="http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#aef6d7357f7d5792329a8c5fea5a5070d">moveit_commander: moveit_commander.move_group.MoveGroupCommander Class Reference</a><br><Div Align="right" style="font-size:xx-small"><a href="http://romberg-iso8.blogspot.com/">powered by swordsmith</a></div>