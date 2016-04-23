# simple_head
## A ROS controller for the Cornell Dynamixel tablet head robot 

You should be able to run the nodes in this package by simply launching the [`simple_head.launch`](launch/simple_head.launch) file. Depends on [dynamixel_controller](http://wiki.ros.org/dynamixel_controllers)

```
roslaunch simple_head.launch
```

### Requesting pose

You request a pose by sending a `simple_head/PoseCommand` message to the `/goto_pose` topic. The message has two fields: a ROS duration (the time it should take to go to the pose) and a string for the pose. 

For example, this will request pose `up_left` with a movement duration of three seconds:

```
rostopic pub -1 /goto_pose simple_head/PoseCommand -- '[3.0,0.0]' 'up_left'
```

**Generally Durations of less than a second are not advised.**

### Adding poses
Just add poses in the [`poses.yaml`](config/poses.yaml) config file. Restart the system. You should be able to send the new pose name on the `/goto_pose` topic.

### Common issues
- Make sure that you have the right USB port (usualy `/dev/ttyUSB0` but might be different on your machine).

### Obvious next steps
- Use `SYNC WRITE` in joint_trajectory 
