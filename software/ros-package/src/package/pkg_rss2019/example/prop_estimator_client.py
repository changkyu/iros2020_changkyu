import os, sys
import rospy
import cv2
from pkg_rss2019.srv import add_object_srv, estimate_prop_srv, plan_pushing_srv

from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D

import matplotlib.pyplot as plt

if __name__ == "__main__":
    bridge = CvBridge()
    rospy.wait_for_service('/pkg_rss2019/add_object')
    rospy.wait_for_service('/pkg_rss2019/estimate_prop')
    rospy.wait_for_service('/pkg_rss2019/plan_pushing')
    try:
        add_object    = rospy.ServiceProxy('/pkg_rss2019/add_object',    add_object_srv)
        estimate_prop = rospy.ServiceProxy('/pkg_rss2019/estimate_prop', estimate_prop_srv)
        plan_pushing  = rospy.ServiceProxy('/pkg_rss2019/plan_pushing',  plan_pushing_srv)

        obj = 'hammer'

        print(os.path.abspath('../../../../../../dataset/objects/'+obj+'_label.pgm'))

        img = cv2.imread('../../../../../../dataset/objects/'+obj+'_label.pgm',cv2.IMREAD_UNCHANGED)
        msg_img = bridge.cv2_to_imgmsg(img, "8UC1")
        add_object(obj, msg_img, 0.53/640.0)

        begin = Pose2D()
        begin.x = 0.477
        begin.y = 0.021
        begin.theta = 0
        final = Pose2D()
        final.x = 0.583
        final.y = 0.031
        final.theta = 0.202
        pos = Pose2D()
        pos.x = 0.460
        pos.y = 0.094
        vec = Pose2D()
        vec.x = 0.10
        vec.y = 0
        duration = 2
        estimate_prop(obj,begin,final,pos,vec,duration)

        init = Pose2D()
        init.x = 0.5
        init.y = 0
        init.theta = 0
        goal = Pose2D()
        goal.x = 0.5
        goal.y = 0.5
        goal.theta = 1.78
        res = plan_pushing(obj, init, goal, 1, None, 0)
        print(res)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e