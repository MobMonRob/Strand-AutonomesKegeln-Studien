# Strand-AutonomesKegeln-Studien
#
```console
roscore
git clone https://github.com/MobMonRob/Strand-AutonomesKegeln-Studien
ln -s ~/Strand-AutonomesKegeln-Studien/ros/* ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
~/catkin_ws/devel/setup.bash
rosrun sphero_bolt_node bolt_node.py
```

