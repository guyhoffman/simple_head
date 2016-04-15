#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from dynamixel_controllers.srv import SetSpeed, SetTorqueLimit


def set_config(dofs, param_name, type, value):
    """
    Set a configuration parameter in a DoF controller

    :param dofs: list of dofs to control
    :param param_name: the parameter to set
    :param type: ROS service type (i.e. SetSpeed)
    :param value: value to set the parameter to
    """
    service_names = ['/' + d + '_controller/set_'+param_name for d in dofs]
    for sn in service_names:
        rospy.wait_for_service(sn)
    services = [rospy.ServiceProxy(sn, type) for sn in service_names]
    for s in services:
        s.call(value)

def set_default_config(dofs, param_name, type):
    """
    Get a value from the parameter server and set the configuation in the controller

    :param dofs: list of dofs to config
    :param param_name: name of config parameter to set
    :param type: ROS service type (i.e. SetSpeed)
    """
    param = rospy.get_param("~pose_"+param_name)
    set_config(dofs, param_name, type, param)

def goto_pose(pose):
    """
    Go to pose by publishing command to each controller topic

    :param pose: pose dictionary key in param server
    """
    if poses.has_key(pose):
        for dof in poses[pose].keys():
            pubs[dof].publish(poses[pose][dof])
        pass
    else:
        rospy.logerr("No such pose: %s " % pose)

# Callback basically just extracts the data from
# the message and goes to that pose
def pose_request(msg_data):
    """
    Callback for topic subscriber. Baiscally extracts the data from the
    msg and sends it to goto_pose()
    :param msg_data: ROS message data
    :return:
    """
    print "Requested pose %s" % msg_data
    goto_pose(msg_data.data)

if __name__ == '__main__':

    # Initialize Node
    rospy.init_node("goer")

    # Poses and DoFs from private params
    poses = rospy.get_param("~poses")
    dofs = poses['zero'].keys()

    # Create Publisher for each DoF
    pubs = {}
    for d in dofs:
        controller = '/'+d+'_controller/command'
        pubs[d] = rospy.Publisher(controller, Float64)

    # Start by going slowly to zero
    set_config(dofs, 'speed', SetSpeed, 0.05)
    goto_pose('zero')

    # Set Motor Speed and Torque Limit from param file
    set_default_config(dofs, "speed", SetSpeed)
    set_default_config(dofs, "torque_limit", SetTorqueLimit)

    # Subscribe to the "goto_pose" topic and wait for String msgs
    rospy.Subscriber("goto_pose", String, pose_request)
    rospy.spin()


