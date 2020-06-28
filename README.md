# Simple Lane Detection for Virat

Package for lane detection

## Usage

* Launch Gazebo and Rviz

```bash
roslaunch virat_gazebo igvc_world.launch
roslaunch virat_gazebo virat_rviz.launch
```

* Launch node
_Important_: Node must be run in a namespace. Parameters should also be loaded here using rosparam.   
The node subscribes to ~/image_color and and publishes to ~/points2

```bash
ROS_NAMESPACE=/camera_topic/left roslaunch virat_lanes lanes_mono.launch
```

**Optional**

* Dynamic reconfigure

First in the parameters, set dynamic_reconfigure to yes, then:
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## Parameters


## Note
(project-TARIV)[https://github.com/project-TARIV/virat_lanes_mono] and (Team-Clueless)[https://github.com/Team-Clueless/lanes_mono] are synced using this (with this.)[https://gist.github.com/rvl/c3f156e117e22a25f242]
