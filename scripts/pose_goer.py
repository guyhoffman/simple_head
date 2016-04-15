#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from dynamixel_controllers.srv import SetSpeed, SetTorqueLimit
from genpy.message import Message


def set_config(dofs, param_name, type, value):
    service_names = ['/' + d + '_controller/set_'+param_name for d in dofs]
    for sn in service_names:
        rospy.wait_for_service(sn)
    services = [rospy.ServiceProxy(sn, type) for sn in service_names]
    for s in services:
        s.call(value)

def set_default_config(dofs, param_name, type):
    param = rospy.get_param("~pose_"+param_name)
    set_config(dofs, param_name, type, param)


def goto_pose(pose):
    print "Requested pose %s" % pose
    if poses.has_key(pose):
        for dof in poses[pose].keys():
            pubs[dof].publish(poses[pose][dof])
        pass
    else:
        rospy.logerr("No such pose: %s " % pose)

def pose_request(msg_data):
    goto_pose(msg_data.data)

if __name__ == '__main__':

    rospy.init_node("goer")

    poses = rospy.get_param("~poses")
    dofs = poses['zero'].keys()

    pubs = {}
    for d in dofs:
        controller = '/'+d+'_controller/command'
        pubs[d] = rospy.Publisher(controller, Float64)


    set_config(dofs, 'speed', SetSpeed, 0.05)
    goto_pose('zero')

    # Set Motor Speed and Torque Limit from Config File
    set_default_config(dofs, "speed", SetSpeed)
    set_default_config(dofs, "torque_limit", SetTorqueLimit)


    rospy.Subscriber("goto_pose", String, pose_request)
    rospy.spin()


