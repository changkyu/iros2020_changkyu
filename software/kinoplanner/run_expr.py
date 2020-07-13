#!/usr/bin/env python
import os, sys
import argparse
import pickle
import time
import cv2
import numpy as np

from KinoPlanner import KinoPlanner
from CellPhysicsPlanner import CellPhysicsPlanner
from CellPhysicsPlanner import Pose2D

dir_root = os.path.abspath('..')
if os.path.abspath(dir_root) not in sys.path:
    sys.path.append(dir_root)
from cellphysics import params
from simulator import SimBullet
from simulator import distance_angle
from cellphysics import CellPhysicsBase

inits = [[-0.00141057 + 0.63,  0.06213397, -1.23017404],
         [-0.02133907 + 0.63, -0.00316645, -0.62773775],
         [-0.08921939 + 0.63,  0.09992869,  2.43793017],
         [-0.27228148 + 0.63,  0.01818074, -1.84462578],
         [ 0.09845183 + 0.63,  0.19546217, -2.63433396],
         [ 0.00910631 + 0.63,  0.00905819,  2.22740577],
         [ 0.21545612 + 0.63,  0.00312732, -1.9830923 ],
         [-0.31053559 + 0.63,  0.10167429, -2.16450234],
         [-0.07953161 + 0.63, -0.25156633,  1.66726542],
         [-0.0594626  + 0.63, -0.26408041, -0.39741391]]

goals = [[-0.2426832  + 0.63, -0.10374377, -2.47021447],
         [ 0.28254698 + 0.63,  0.13652749, -3.01254587],
         [ 0.02765625 + 0.63,  0.07294656,  0.52700383],
         [ 0.08143976 + 0.63, -0.1857621 ,  2.21383141],
         [-0.04914379 + 0.63,  0.32043622,  0.2220155 ],
         [ 0.03449074 + 0.63,  0.05797856,  1.16675392],
         [-0.0437046  + 0.63,  0.16962886, -0.24203532],
         [-0.03774509 + 0.63,  0.13878114,  0.58572018],
         [-0.15263483 + 0.63, -0.10376166,  2.59806677],
         [-0.09881254 + 0.63, -0.09580956, -1.7375131 ]]

objects = ['book','box','crimp','hammer','ranch','snack','toothpaste','windex']
methods_planonly = ['diff','bfs']
methods_closed = ['diffclosed','noneclosed']

if __name__=='__main__':

    parser = argparse.ArgumentParser(description='Test')
    parser.add_argument('-o','--outdir',type=str,  default='./res',help='output directory')
    parser.add_argument('-p','--param', type=int,  default=-1,    help='idx params')
    parser.add_argument('-e','--expr',  type=str,  required=True, help='experiment', choices=['planonly','closed'])
    parser.add_argument('--skip',       type=bool, default=False, help='skip')
    parser.add_argument('--method',     type=str,  nargs='+',     help='method',  choices=methods_planonly+methods_closed)
    parser.add_argument('--objects',    type=str,  nargs='+',     help='objects', choices=objects)
    parser.add_argument('--sets',       type=int,  default=-1,    help='sets')
    parser.add_argument('--debug',      type=bool, default=False, help='visualization')
    args = parser.parse_args()

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    if args.objects is None or len(args.objects)==0:
        args.objects = objects

    if args.method is None or len(args.method)==0:
        if args.expr=='planonly':
            args.method = methods_planonly
        else:
            args.method = methods_closed

    for method in args.method:
        if args.expr=='planonly':
            if method not in methods_planonly:
                print('[Error] Invalid method: {} (choose from {})'.format(method,methods_planonly))
                exit(0)
        elif args.expr=='closed':
            if method not in methods_closed:
                print('[Error] Invalid method: {} (choose from {})'.format(method,methods_closed))
                exit(0)

    if args.param > 0:
        args.param = [args.param - 1]
    else:
        args.param = range(10)

    initgoals = zip(inits, goals)

    if args.sets > 0:
        args.sets = [args.sets - 1]
    else:
        args.sets = range(len(initgoals))

    meter_per_pixel = 0.53/640.0

    if args.expr=='planonly':
        for idx_param in args.param:
            for obj in args.objects:
                fp_label = '../../dataset/objects/%s_label.pgm' % obj
                param = params[obj][idx_param]
                for idx_initgoal in args.sets:
                    init = initgoals[idx_initgoal][0]
                    goal = initgoals[idx_initgoal][1]
                    for method in args.method:

                        fp_save = os.path.join(args.outdir,                            
                                               '%s_%s_param%d_initgoal%d.pkl'%\
                                               (method,obj,idx_param,idx_initgoal))
                        if args.skip and os.path.isfile(fp_save):
                            print('[SKIP] %s' % fp_save)
                            continue

                        print('method: {}, param: {}, init: {}, goal: {}'.format(method,param,init,goal))

                        time_beg = time.time()
                        planner = KinoPlanner(0.01,[0.20,0.15,0.10,0.05,0.03,0.02,0.01],debug=False)            
                        planner.addObject(obj,fp_label, param['masses'], meter_per_pixel=meter_per_pixel)
                        path, actions = planner.plan(obj,init,goal, method=method)
                        time_end = time.time()
                        print('time: {}'.format(time_end - time_beg))
                        print('# of actions: {}'.format(len(actions)))

                        summary = {}
                        summary['init'] = init
                        summary['goal'] = goal
                        summary['path'] = path
                        summary['actions'] = actions
                        summary['n_actions'] = len(actions)
                        summary['time'] = (time_end - time_beg)
                        summary['n_sims'] = len(planner.trans_objs[obj])
                        print(summary['n_sims'])

                        with open(fp_save,'wb') as f:
                            pickle.dump(summary, f, protocol=2)

    elif args.expr=='closed':

        for idx_param in args.param:
            for obj in args.objects:
                fp_label = '../../dataset/objects/%s_label.pgm' % obj
                img = cv2.imread(fp_label, cv2.IMREAD_UNCHANGED)
                param = params[obj][idx_param]
                for idx_initgoal in args.sets:
                    init = initgoals[idx_initgoal][0]
                    goal = initgoals[idx_initgoal][1]
                    for method in args.method:
                        fp_save = os.path.join(args.outdir,                            
                                               '%s_%s_param%d_initgoal%d.pkl'%\
                                               (method,obj,idx_param,idx_initgoal))
                        if args.skip and os.path.isfile(fp_save):
                            print('[SKIP] %s' % fp_save)
                            continue

                        print('method: {}, param: {}, init: {}, goal: {}'.format(method,param,init,goal))
                        time_beg = time.time()

                        planner = CellPhysicsPlanner(obj, img, [5-0.1]*4, meter_per_pixel)

                        sim = SimBullet(gui=False)
                        sim.setupCamera(0.75,-90,-89.99, 0.63,0,0)
                        sim.addObject('plane','plane',[1,1,1,1])
                        sim.addObject('finger','finger',[1,1,0,1])
                        cells = CellPhysicsBase.createCells(img,param['masses'],meter_per_pixel=meter_per_pixel)
                        sim.addObject(obj, cells, [0.5,0.5,0.5,1], 0,0,0)
                        sim.setObjectPose(obj,goal[0],goal[1],goal[2])
                        cellpos_goal = sim.getCellBodyPose(obj)
                        sim.setObjectPose(obj,init[0],init[1],init[2])
    
                        history = {'path':[], 'actions':[]}
                        while True:
                            x0,y0,yaw0 = sim.getObjectPose(obj)
                            cellpos0 = sim.getCellBodyPose(obj)
                            err_xy = np.linalg.norm([goal[0]-x0,goal[1]-y0])
                            err_yaw = distance_angle(goal[2],yaw0)
                            err_cell = CellPhysicsBase.distanceCellPos(cellpos_goal, cellpos0)
                            cellinfos = planner.getProperties()
                            cellinfos_copy = [info.copy() for info in cellinfos]
                            history['path'].append({'x':x0,'y':y0,'yaw':yaw0,'cellinfos':cellinfos_copy, 
                                                    'err_xy':err_xy,'err_yaw':err_yaw,'err_cell':err_cell})
                                    
                            print('err_cell: %.3f err_xy: %.3f err_yaw: %.3f' %(err_cell,err_xy,err_yaw))
                            if err_cell < 0.02:
                                print('[SUCCESS]')
                                break

                            path, actions = planner.plan([x0,y0,yaw0], goal)
                            action = actions[0]
                            history['actions'].append(action.copy())

                            planner.planner.applyAction(sim,obj,x0,y0,yaw0,action)
                            x1,y1,yaw1 = sim.getObjectPose(obj)

                            if method=='diffclosed':
                                for key, value in action.items() if type(action)==dict else action:
                                    if key=='pos':
                                        pos_push = [0,0]
                                        pos_push[0] = np.cos(yaw0)*value[0]-np.sin(yaw0)*value[1] + x0
                                        pos_push[1] = np.sin(yaw0)*value[0]+np.cos(yaw0)*value[1] + y0
                                    elif key=='velocity':
                                        vec_push = [0,0]
                                        vec_push[0] = np.cos(yaw0)*value[0]-np.sin(yaw0)*value[1]
                                        vec_push[1] = np.sin(yaw0)*value[0]+np.cos(yaw0)*value[1]
                                    elif key=='duration':
                                        duration = value
                                vec_push[0] = vec_push[0]*duration
                                vec_push[1] = vec_push[1]*duration

                                pos_begin = Pose2D(x0,y0,yaw0)
                                pos_final = Pose2D(x1,y1,yaw1)
                                pos_push = Pose2D(pos_push[0],pos_push[1],0)
                                vec_push = Pose2D(vec_push[0],vec_push[1],0)
                                planner.addObservation(pos_begin, pos_final, pos_push, vec_push, duration)

                            x0,y0,yaw0 = (x1,y1,yaw1)

                        time_end = time.time()
                        print('time: {}'.format(time_end - time_beg))
                        print('# of actions: {}'.format(len(actions)))

                        summary = {}
                        summary['init'] = init
                        summary['goal'] = goal
                        summary['path'] = history['path']
                        summary['actions'] = history['actions']
                        summary['n_actions'] = len(history['actions'])
                        summary['time'] = (time_end - time_beg)

                        with open(fp_save,'wb') as f:
                            pickle.dump(summary, f, protocol=2)