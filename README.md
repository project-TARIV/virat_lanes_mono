# virat_lanes

Package for lane detection

## Usage

* Launch Gazebo simulation

```bash
roslaunch virat_gazebo igvc_world.launch
```

* Launch Rviz

```bash
roslaunch virat_gazebo virat_rviz.launch
```

* Launch node

```bash
roslaunch virat_lanes lanes_mono.launch
```

**Optional**

* Dynamic reconfigure

Enable dyn reconfigure from params
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
