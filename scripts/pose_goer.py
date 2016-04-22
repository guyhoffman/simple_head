#!/usr/bin/env python

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from dynamixel_controllers.srv import SetSpeed, SetTorqueLimit
from trajectory_msgs.msg import JointTrajectoryPoint

from simple_head.msg import PoseCommand

CONFIG_SRV_TYPES = {"speed": SetSpeed, "torque_limit": SetTorqueLimit}


class PoseGoer:
    def __init__(self):

        # Initialize Node
        rospy.init_node("goer")

        # Poses and DoFs from param file
        self.poses = {}
        self.dofs = []

        self.load_poses()

        # Connect to Action Server
        self.ac = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.ac.wait_for_server()
        rospy.loginfo('Connected to joint trajectory action!')

        # Start by going slowly to zero
        self.set_config('speed', 0.05)
        self.goto_pose('zero', rospy.Duration.from_sec(5))

        # Set (potentially higher) Motor Speed and Torque Limit from param file
        self.set_default_configs()

        # Subscribe to the "goto_pose" topic and wait for String msgs
        rospy.Subscriber("goto_pose", PoseCommand, self.pose_request)

    def load_poses(self):
        """
        Converts each pose from the YAML file into a goal and creates members poses and dofs:
        poses is a dictionary mapping "name"->goal, dofs is the list of joints
        """
        param_poses = rospy.get_param("~poses")
        for name, pp in param_poses.iteritems():
            goal = FollowJointTrajectoryGoal()
            self.dofs = pp.keys()

            positions = JointTrajectoryPoint()
            positions.positions = pp.values()

            goal.trajectory.joint_names = self.dofs
            goal.trajectory.points.append(positions)

            self.poses[name] = goal

    def set_config(self, param, value):
        """
        Set a configuration parameter in a set of DoF controllers

        :param param: the parameter to set
        :param value: value to set the parameter to
        """
        service_names = ['/' + dof + '_controller/set_' + param for dof in self.dofs]
        for sn in service_names:
            rospy.wait_for_service(sn)
            service = rospy.ServiceProxy(sn, CONFIG_SRV_TYPES[param])
            service.call(value)

    def set_default_configs(self):
        """
        Get values from the parameter server and set the configuration in the controller
        """
        for param in CONFIG_SRV_TYPES.keys():
            value = rospy.get_param("~pose_" + param)
            self.set_config(param, value)

    def goto_pose(self, pose, duration):
        """
        Go to pose by publishing command to each controller topic

        :param pose: pose dictionary key in param server
        :param duration: duration for trajectory
        """
        if pose in self.poses:
            goal = self.poses[pose]
            goal.trajectory.header.stamp = rospy.Time.now()
            goal.trajectory.points[0].time_from_start = duration  # Assuming only one point per trajectory
            self.ac.send_goal(goal)
        else:
            rospy.logerr("No such pose: %s " % pose)

    # Subscriber callback basically just extracts the data from
    # the message and goes to that pose
    def pose_request(self, msg):
        """
        Callback for topic subscriber. Basically extracts the data from the
        msg and sends it to goto_pose()
        :param msg: ROS message data
        :return:
        """
        print "Requested pose %s" % msg.pose
        self.goto_pose(msg.pose, msg.duration)

if __name__ == '__main__':
    try:
        goer = PoseGoer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Bah-bye...")
