# simple_head
## A ROS controller for the Cornell Dynamixel tablet head robot 

You should be able to run the nodes in this package by simply launching the [`simple_head.launch`](launch/simple_head.launch) file. 

### Adding poses
Just add poses in the [`poses.yaml`](config/poses.yaml) config file. Restart the system. You should be able to send the new pose name on the `/goto_pose` topic.

### Common issues
- Make sure that you have the right USB port (usualy `/dev/ttyUSB0` but might be different on your machine).
