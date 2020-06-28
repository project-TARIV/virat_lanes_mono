# Simple Lane Detection for Virat

Package for lane detection

## Usage

* Launch Gazebo and Rviz

```bash
roslaunch virat_gazebo igvc_world.launch
roslaunch virat_gazebo virat_rviz.launch
```

* Launch node
_Important_: Launch file must be run in a namespace. 
The system subscribes to ~/image_raw and and publishes to ~/points2

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
- In the launchfile:
  - image_proc (bool, default 1): Whether to spawn the image_processing node
  - param_file (string, default $(find lanes_mono)/params/sample.yaml): The parameter file to load
  
- In param file: See [sample.yaml](params/sample.yaml)



## Note
[project-TARIV](https://github.com/project-TARIV/virat_lanes_mono) and [Team-Clueless](https://github.com/Team-Clueless/lanes_mono) are synced using this [with this.](https://gist.github.com/rvl/c3f156e117e22a25f242)
