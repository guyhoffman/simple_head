#!/usr/bin/env python

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from dynamixel_controllers.srv import SetSpeed, SetTorqueLimit
from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectoryPoint


def set_config(dofs_, param_, type_, value_):
    """
    Set a configuration parameter in a DoF controller

    :param dofs_: list of dofs to control
    :param param_: the parameter to set
    :param type_: ROS service type (i.e. SetSpeed)
    :param value_: value to set the parameter to
    """
    service_names = ['/' + dof + '_controller/set_' + param_ for dof in dofs_]
    for sn in service_names:
        rospy.wait_for_service(sn)
    services = [rospy.ServiceProxy(sn, type_) for sn in service_names]
    for s in services:
        s.call(value_)


def set_default_config(dofs_, param_, type_):
    """
    Get a value from the parameter server and set the configuration in the controller

    :param dofs_: list of dofs to config
    :param param_: name of config parameter to set
    :param type_: ROS service type (i.e. SetSpeed)
    """
    param = rospy.get_param("~pose_" + param_)
    set_config(dofs_, param_, type_, param)


def goto_pose(pose):
    """
    Go to pose by publishing command to each controller topic

    :param pose: pose dictionary key in param server
    """
    if pose in poses:
        ac.send_goal(poses[pose])
    else:
        rospy.logerr("No such pose: %s " % pose)


# Callback basically just extracts the data from
# the message and goes to that pose
def pose_request(msg_data):
    """
    Callback for topic subscriber. Basically extracts the data from the
    msg and sends it to goto_pose()
    :param msg_data: ROS message data
    :return:
    """
    print "Requested pose %s" % msg_data
    goto_pose(msg_data.data)


def load_poses():
    """
    Converts each pose from the YAML file into a goal

    :return: (poses, dofs): poses is a dictionary mapping "name"->goal, dofs is the list of joints
    """
    param_poses = rospy.get_param("~poses")
    print param_poses
    poses_ = {}
    dofs_ = []
    for name, pp in param_poses.iteritems():
        goal = FollowJointTrajectoryGoal()
        dofs_ = pp.keys()
        goal.trajectory.joint_names = dofs_
        positions = JointTrajectoryPoint()
        positions.positions = pp.values()
        positions.time_from_start = rospy.Duration.from_sec(3.0)  # Should make parameter in Topic msg
        goal.trajectory.points.append(positions)
        goal.trajectory.header.stamp = rospy.Time.now()
        print goal
        poses_[name] = goal

    return poses_, dofs_


if __name__ == '__main__':

    # Initialize Node
    rospy.init_node("goer")

    # Poses and DoFs from private params
    poses, dofs = load_poses()

    # Create Publisher for each DoF
    pubs = {}
    for d in dofs:
        controller = '/' + d + '_controller/command'
        pubs[d] = rospy.Publisher(controller, Float64)

    ac = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo('Waiting for joint trajectory action')
    ac.wait_for_server()
    rospy.loginfo('Found joint trajectory action!')

    # Start by going slowly to zero
    set_config(dofs, 'speed', SetSpeed, 0.05)
    goto_pose('zero')

    # Set Motor Speed and Torque Limit from param file
    set_default_config(dofs, "speed", SetSpeed)
    set_default_config(dofs, "torque_limit", SetTorqueLimit)

    # Subscribe to the "goto_pose" topic and wait for String msgs
    rospy.Subscriber("goto_pose", String, pose_request)
    rospy.spin()
