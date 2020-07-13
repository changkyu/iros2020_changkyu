#!/usr/bin/env python
import glob
import pickle
import cv2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import os, sys
dir_current, _ = os.path.split(os.path.abspath(__file__))
dir_software = os.path.join(dir_current,'../../../../..')
if os.path.abspath(dir_software) not in sys.path:
    sys.path.append(dir_software)

from cellphysics import DiffPhysics
from kinoplanner import ProbPhysicsPlanner
from simulator import distance_angle
from simulator import SimBullet

from pkg_rss2019.srv import add_object_srv,add_object_srvResponse
from pkg_rss2019.srv import estimate_prop_srv,estimate_prop_srvResponse
from pkg_rss2019.srv import plan_prob_pushing_srv,plan_prob_pushing_srvResponse

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge,CvBridgeError

bridge = CvBridge()
planner = {}
pub_plan_debug = None
pub_prop_debug = None

def handle_add_object(req):

    dir_sav = rospy.get_param('/changkyu/pkg_rss2019/prob_planner_srv/models_path')

    print('\n[Add Object] %s' % req.name)
    if req.name not in planner:
        try:
            image = bridge.imgmsg_to_cv2(req.image, "8UC1")
        except CvBridgeError as e:
            print(e)

        models = []
        """
        #files = glob.glob(dir_sav+'/'+req.name+'*.pkl')
        files = [dir_sav+'/'+req.name+'_seed1.pkl',
                 dir_sav+'/'+req.name+'_seed4.pkl']
        for file in files:
            print('load ' + file)
            f = open(file,'rb')
            models.append(pickle.load(f))
            f.close()
        #planner[req.name] = ProbPhysicsPlanner(req.name,image,models,req.meter_per_pixel)
        """
        #sim = SimBullet(gui=False)
        #cells = DiffPhysics.createCells(image,[30,1],meter_per_pixel=req.meter_per_pixel)
        #sim.addObject(req.name, cells, [0.5,0.5,0.5,1], 0, 0, 0)
        #models.append({'cellinfo':sim.getCellBodyProperties(req.name), 'error_cell':0})
        #models.append({'cellinfo':[3,1], 'error_cell':0})
        models.append({'cellinfo':[3.5,1], 'error_cell':0})
        #models.append({'cellinfo':[1,1], 'error_cell':0})

        planner[req.name] = ProbPhysicsPlanner(req.name,image,models,req.meter_per_pixel,
                                               contact_resolution=0.02,
                                               pushing_dists=[0.20, 0.15, 0.10, 0.05, 0.03, 0.015] )
        for name_i in planner[req.name].planner.name2names[req.name]:
            if len(planner[req.name].planner.trans_objs[name_i])==0:
                planner[req.name].planner.buildTransitionTable(name_i)

    else:
        print('Already exists.')
    return []

def handle_estimate_prop(req):

    print('\n[Estimate Properties] %s' % req.name)
    if req.name not in planner:
        print('[Error] Please add object first')
        return []

    planner[req.name].addObservation(req.pos_begin, 
                                     req.pos_final, 
                                     req.pos_push, 
                                     req.vec_push, 
                                     req.duration)
    img_prop = planner[req.name].drawProperties()
    msg_prop = bridge.cv2_to_imgmsg(img_prop)
    pub_prop_debug.publish(msg_prop)
    print('done')
    return []

def handle_plan_pushing(req):

    global bridge
    global planner
    global pub_plan_debug

    init = [req.init.x, req.init.y, req.init.theta]
    #goals = [[req.goal.x, req.goal.y, req.goal.theta],
    #         [req.goal.x, req.goal.y, distance_angle(req.goal.theta,np.pi)]]
    goals = [[goal.x,goal.y,goal.theta] for goal in req.goals]

    if True:
        goals_sel = planner[req.name].selectGoals(req.name, goals)
        goals_sel = goals
        print('Selected Goals: ', goals_sel)

        dist_yaw_min = np.inf
        for goal_sel in goals_sel:
            dist_yaw = abs(distance_angle(goal_sel[2],req.init.theta))
            if dist_yaw_min > dist_yaw:
                dist_yaw_min = dist_yaw
                goal = goal_sel
    else:
        goal = goals[0]
    print('Selected Goal: ', (goal[0],goal[1],goal[2]*180/np.pi))

    print('\n[Planning] %s' % req.name)
    print('{} to {}'.format(init,goal))
    if req.name not in planner:
        print('[Error] Please add object first')
        return []

    plans = planner[req.name].plan(init,goal)

    begs = []
    ends = []
    actions = []
    nexts = []    
    for plan in plans:
        action = plan['action']

        x,y,yaw = init
        for key, value in action.items() if type(action)==dict else action:
            if key=='pos':
                pos = [0,0]
                pos[0] = np.cos(yaw)*value[0]-np.sin(yaw)*value[1] + x
                pos[1] = np.sin(yaw)*value[0]+np.cos(yaw)*value[1] + y
            elif key=='velocity':
                velocity = [0,0]
                velocity[0] = np.cos(yaw)*value[0]-np.sin(yaw)*value[1]
                velocity[1] = np.sin(yaw)*value[0]+np.cos(yaw)*value[1]
            elif key=='duration':
                duration = value

        pose0 = Pose2D()
        pose0.x, pose0.y = pos
        pose1 = Pose2D()
        pose1.x, pose1.y = np.array(pos) + np.array(velocity)
        begs.append(pose0)
        ends.append(pose1)
        pose = Pose2D()
        pose.x, pose.y, pose.theta = plan['nexts'][0][1:4]
        nexts.append(pose)

    path = [init, plans[0]['nexts'][0][1:4]]
    actions.append(plans[0]['action'])

    img_plan = planner[req.name].drawPlan(path, actions)
    img_plan = cv2.cvtColor(img_plan, cv2.COLOR_RGBA2BGR)
    msg_plan = bridge.cv2_to_imgmsg(img_plan)
    pub_plan_debug.publish(msg_plan)
    print('done')

    return plan_prob_pushing_srvResponse(nexts,begs,ends)

if __name__=='__main__':
    rospy.init_node('prob_planner_srv')

    rospy.Service('/pkg_rss2019/add_object',    add_object_srv,    handle_add_object)
    rospy.Service('/pkg_rss2019/estimate_prop', estimate_prop_srv, handle_estimate_prop)
    rospy.Service('/pkg_rss2019/plan_prob_pushing',  plan_prob_pushing_srv,  handle_plan_pushing)
    
    pub_plan_debug = rospy.Publisher('/pkg_rss2019/markers/plan/image', Image, queue_size=10)
    pub_prop_debug = rospy.Publisher('/pkg_rss2019/markers/prop/image', Image, queue_size=10)

    print "[Ready]"
    rospy.spin()