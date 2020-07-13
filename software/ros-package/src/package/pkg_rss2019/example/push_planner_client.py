import sys
import rospy
import cv2
from pkg_rss2019.srv import push_planner_srv

from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D

import matplotlib.pyplot as plt

if __name__ == "__main__":
    bridge = CvBridge()
    rospy.wait_for_service('/pkg_rss2019/push_planner_srv')
    try:
        push_planner = rospy.ServiceProxy('/pkg_rss2019/push_planner_srv', push_planner_srv)
        img = cv2.imread('/home/cm1074/changkyu/rss2019/dataset/sims/hammer_label.pgm',cv2.IMREAD_UNCHANGED)
        msg_img = bridge.cv2_to_imgmsg(img, "8UC1")
        init = Pose2D()
        init.x = 0.5
        init.y = 0
        init.theta = 0
        goal = Pose2D()
        goal.x = 0.5
        goal.y = 0.5
        goal.theta = 1.78

        res = push_planner('target', msg_img, init, goal, 0.35/500.0)
        print(res)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e