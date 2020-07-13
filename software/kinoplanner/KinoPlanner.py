import os, sys
import math
import numpy as np
import cv2
import time
import progressbar
import multiprocessing
from multiprocessing import Pool

from ompl import base as ob
from ompl import geometric as og

dir_root = os.path.abspath('..')
if os.path.abspath(dir_root) not in sys.path:
    sys.path.append(dir_root)
from simulator import SimBullet
from simulator import distance_angle
from simulator.SimBullet import quaterion_to_euler

from cellphysics import CellPhysicsBase

import ompl.util
ompl.util.setLogLevel(ompl.util.LogLevel.LOG_ERROR)

# for debug
import matplotlib.pyplot as plt

def computeTransition_multiproc(args):
    name, cells, action, name_actor, type_actor, dims_actor = args

    sim = SimBullet(gui=False)
    sim.addObject('plane','plane')
    sim.addObject(name, cells, [0,0,0,1], 0,0,0)
    sim.addObject(name_actor,type_actor,[0,0,0,1])

    KinoPlanner.applyAction(sim,name,0,0,0,action,name_actor,dims_actor)
    x,y,yaw = sim.getObjectPose(name)
    cellpos = sim.getCellBodyPose(name)
    return {'vec':[x,y],'yaw':yaw,'cellpos':cellpos}

def isActionStable_multiproc(args):
    name, cells, x, y, yaw, action, name_actor, type_actor, dims_actor = args

    sim = SimBullet(gui=False)
    sim.addObject('plane','plane')
    sim.addObject(name, cells, [0,0,0,1], 0,0,0)
    sim.addObject(name_actor,type_actor,[0,0,0,1])
    sim.setObjectPose(name,x,y,yaw)

    (x0,y0,z0),q0 = sim.getObjectPose(name,ndims=3)
    KinoPlanner.applyAction(sim, name,x,y,yaw, \
                            action,name_actor,dims_actor)
    (x1,y1,z1),q1 = sim.getObjectPose(name,ndims=3)

    q0 = q0.normalised
    q1 = q1.normalised
    dist_q = 1 - np.dot(q0.q,q1.q)**2

    return z1>0 or dist_q<0.003 # <10degree

class KinoPlanner:

    def __init__(self, setup=None, contact_resolution=0.02, pushing_dists=[0.02,0.04,0.06,0.08,0.10], debug=False, use_multiproc=False):

        self.name2cells = {}

        self.space = ob.SE2StateSpace()
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(-1)
        bounds.setHigh(1)
        self.space.setBounds(bounds)
        self.si = ob.SpaceInformation(self.space)
        
        self.name_actor = 'finger'
        self.type_actor = 'finger'
        self.color_actor = [1,1,0,1]
        self.dims_actor = [0.03, 0.03]

        self.name2names = {}
        self.name2probs = {}

        self.sim_cols = {}
        self.sim_stbs = {}
        self.sim_objs = {}
        self.stbs_objs = {}
        self.trans_objs = {}
        self.actions_objs = {}
        self.n_actions_objs = {}
        self.uvec_contact2com = {}
        self.com_objs = {}

        if setup is None:
            setup = {'table_type','table_round'}
        self.setup = setup
        self.setup['table_radius'] = SimBullet.get_shapes()[self.setup['table_type']]['radius']

        self.debug = debug
        if self.debug:
            self.sim_debug = SimBullet(gui=self.debug)
            self.addDefaultObjects(self.sim_debug)

        self.resolution_contact = contact_resolution
        self.pushing_dists = sorted(pushing_dists)
        self.use_multiproc = use_multiproc
        if self.use_multiproc:
            self.pool = Pool(int(multiprocessing.cpu_count()*0.8+0.5))

    def addDefaultObjects(self, sim):
        sim.addObject('table',self.setup['table_type'],[1,1,1,1])
        sim.setObjectPose('table',0.63,0,0)
        sim.addObject(self.name_actor,self.type_actor,self.color_actor)

    def createState(self, x, y, yaw):

        state = ob.State(self.space)
        state().setX(x)
        state().setY(y)
        state().setYaw(yaw)
        return state

    @staticmethod
    def action2key(action):
        return tuple(action.items()) if type(action)==dict else action

    def genActions(self, cells, interval):

        cell_size = cells['cell_size']
        meter_per_pixel = 0.001

        img = np.zeros([500,500],dtype=np.uint8)
        for cell in cells['cells']:
            pt1 = np.array([-cell['pos'][1],-cell['pos'][0]]) - cell_size
            pt2 = pt1 + cell_size*2
            pt1 = (pt1/float(meter_per_pixel) + np.array([img.shape[1],img.shape[0]])*0.5).astype(int)
            pt2 = (pt2/float(meter_per_pixel) + np.array([img.shape[1],img.shape[0]])*0.5).astype(int)
            cv2.rectangle(img, tuple(pt1), tuple(pt2), (255,255,255))

        points = {(1,0):[], (-1,0):[], (0,1):[], (0,-1):[]}

        for c in range(img.shape[1]):
            for r in range(img.shape[0]):
                if img[r,c] > 0:
                    x = (-(r-img.shape[0]*0.5)*meter_per_pixel)
                    y = (-(c-img.shape[1]*0.5)*meter_per_pixel)
                    points[(-1,0)].append((x,y))
                    break

        for c in range(img.shape[1]):
            for r in range(img.shape[0]-1,-1,-1):
                if img[r,c] > 0:
                    x = (-(r-img.shape[0]*0.5)*meter_per_pixel)
                    y = (-(c-img.shape[1]*0.5)*meter_per_pixel)
                    points[(1,0)].append((x,y))
                    break

        for r in range(img.shape[0]):
            for c in range(img.shape[1]):
                if img[r,c] > 0:
                    x = (-(r-img.shape[0]*0.5)*meter_per_pixel)
                    y = (-(c-img.shape[1]*0.5)*meter_per_pixel)
                    points[(0,-1)].append((x,y))
                    break

        for r in range(img.shape[0]):
            for c in range(img.shape[1]-1,-1,-1):
                if img[r,c] > 0:
                    x = (-(r-img.shape[0]*0.5)*meter_per_pixel)
                    y = (-(c-img.shape[1]*0.5)*meter_per_pixel)
                    points[(0,1)].append((x,y))
                    break

        interval_px = int(interval/float(meter_per_pixel))
        for key in points:
            n_points = len(points[key])
            n_contacts = n_points // interval_px - 2
            if n_contacts <= 0:
                n_contacts = 1
            # [KUKA]
            #if key==(0,-1) or key==(0,1):
            #    n_contacts = 1

            i_beg = n_points//2 - (n_contacts-1)//2*interval_px
            i_end = n_points//2 + (n_contacts-1)//2*interval_px

            contacts = []
            for i in range(i_beg,i_end+1,interval_px):
                contacts.append(points[key][i])
            points[key] = contacts

        print('Generating Actions ... ')
        actions = {}
        for direction in points:
            actions[direction] = []
            for pos in points[direction]:
                actions[direction].append([])
                for length in self.pushing_dists:
                    velocity = tuple(np.array(direction) * length)
                    action = {'pos':pos, 'velocity':velocity, 'duration':1.0,
                              'direction':direction,
                              'idx':len(actions[direction])-1,
                              'ldx':len(actions[direction][-1])              }
                    actions[direction][-1].append(action)
            print('direction {:<10}: {:>4} x {:>4} (# of contacts) x (# of pushing length)'.format(direction,len(actions[direction]),len(self.pushing_dists)))
        return actions

    def addObject(self, name, fp_label, masses, meter_per_pixel, color=[0.5,0.5,0.5,1], x=0, y=0, yaw=0, probs=None):

        if type(fp_label)==str:
            img = cv2.imread(fp_label, cv2.IMREAD_UNCHANGED)
        else:
            img = fp_label

        if type(masses[0])==list:
            names = [name+'_'+str(i) for i in range(len(masses))]
            if probs is None:
                probs = [1/float(len(names))] * len(names)
        else:
            names = [name+'_0']
            probs = [1]
            masses = [masses]

        self.name2names[name] = names
        self.name2probs[name] = probs
        for i, name in enumerate(names):

            if type(masses[i][0])==dict:
                cells = CellPhysicsBase.createCells(img,[1,1,1,1],meter_per_pixel=meter_per_pixel)
            else:
                cells = CellPhysicsBase.createCells(img,masses[i],meter_per_pixel=meter_per_pixel)

            self.name2cells[name] = cells
            
            self.sim_cols[name] = SimBullet(gui=False)        
            self.sim_cols[name].addObject(name, cells, color, x, y, yaw)

            self.sim_stbs[name] = SimBullet(gui=False)
            self.sim_stbs[name].addObject(name, cells, color, x, y, yaw)
            self.addDefaultObjects(self.sim_stbs[name])

            self.sim_objs[name] = SimBullet(gui=False)
            self.sim_objs[name].setupCamera(0.50,-90,-89.99)
            self.sim_objs[name].addObject('plane','plane')
            self.sim_objs[name].addObject(name, cells, color, x, y, yaw)
            self.sim_objs[name].addObject(self.name_actor,self.type_actor,self.color_actor)

            if self.debug:
                self.sim_debug.addObject(name, cells, color, x, y, yaw)

            self.trans_objs[name] = {}
            self.actions_objs[name] = self.genActions(cells, self.resolution_contact)
            self.n_actions_objs[name] = sum([len(self.actions_objs[name][key]) for key in self.actions_objs[name]]) * len(self.pushing_dists)
            self.uvec_contact2com[name] = {}

            if type(masses[i][0])==dict:
                cellprop = masses[i]
            else:
                cellprop = self.sim_objs[name].getCellBodyProperties(name)
            self.updateObjectProperties(name, cellprop)

    def updateObjectProperties(self, name, cellprop):

        self.trans_objs[name] = {} # clear 

        self.sim_cols[name].setCellBodyProperties(name, cellprop)
        self.sim_stbs[name].setCellBodyProperties(name, cellprop)
        self.sim_objs[name].setCellBodyProperties(name, cellprop)
        if self.debug:
            self.sim_debug.setCellBodyProperties(name, cellprop)
        
        com = np.array([0,0])
        mass_sum = 0
        self.sim_objs[name].setObjectPose(name,0,0,0)
        cellpos = self.sim_objs[name].getCellBodyPose(name)
        cellprop = self.sim_objs[name].getCellBodyProperties(name)
        for pos, prop in zip(cellpos, cellprop):
            com[0] = pos[0] * prop['mass']
            com[1] = pos[1] * prop['mass']
            mass_sum = mass_sum + prop['mass']
        com[0] = com[0] / float(mass_sum)
        com[1] = com[1] / float(mass_sum)
        self.com_objs[name] = com
        
        for direction in self.actions_objs[name]:
            uvecs = []
            for actions in self.actions_objs[name][direction]:
                vec = com - np.array(actions[-1]['pos'])
                uvecs.append(vec/np.linalg.norm(vec))
            self.uvec_contact2com[name][direction] = np.array(uvecs)

    def updateModelProbabilities(self, name, probs):

        probs_sum = sum(probs)
        self.name2probs[name] = [prob/float(probs_sum) for prob in probs]

    @staticmethod
    def applyAction(sim, name, x, y, yaw, action, name_actor, dims_actor):

        for key, value in action.items() if type(action)==dict else action:
            if key=='pos':
                pos = [0,0]
                pos[0] = np.cos(yaw)*value[0]-np.sin(yaw)*value[1] + x
                pos[1] = np.sin(yaw)*value[0]+np.cos(yaw)*value[1] + y
            elif key=='velocity':
                velocity = [0,0,0]
                velocity[0] = np.cos(yaw)*value[0]-np.sin(yaw)*value[1]
                velocity[1] = np.sin(yaw)*value[0]+np.cos(yaw)*value[1]
            elif key=='duration':
                duration = value

        length = np.linalg.norm(velocity)
        sim.setObjectPose(name,x,y,yaw)
        sim.setObjectPose(name_actor,
            pos[0]-velocity[0]/float(length)*dims_actor[0],\
            pos[1]-velocity[1]/float(length)*dims_actor[1],yaw)

        #duration = duration + self.dims_actor[0] / length

        sim.applyConstantVelocity(name_actor,velocity,duration)

    def getCellPos(self, name, x, y, yaw):

        self.sim_objs[name].setObjectPose(name, x, y, yaw)
        return self.sim_objs[name].getCellBodyPose(name)

    def computeTransition(self, name, action):

        key = self.action2key(action)
        if key in self.trans_objs[name]:
            return self.trans_objs[name][key]

        self.sim_objs[name].setObjectPose(name,0,0,0)
        cellpos0 = self.sim_objs[name].getCellBodyPose(name)
        self.applyAction(self.sim_objs[name],name,0,0,0,action,self.name_actor,self.dims_actor)
        x1,y1,yaw1 = self.sim_objs[name].getObjectPose(name)
        cellpos1 = self.sim_objs[name].getCellBodyPose(name)
        
        dist = CellPhysicsBase.distanceCellPos(cellpos0,cellpos1)
        if dist < 0.001:
            return None

        return {'vec':[x1,y1],'yaw':yaw1,'cellpos':cellpos1}

    def computeTransition_models(self, name, action):

        names = self.name2names[name]
        if self.use_multiproc:
            ret = self.pool.map(computeTransition_multiproc,
                                [(name_i,
                                  self.name2cells[name],
                                  action,
                                  self.name_actor,
                                  self.type_actor,
                                  self.dims_actor) for name_i in names])
        else:
            ret = []
            for name_i in names:
                ret.append(self.computeTransition(name_i, action))

        return ret

    def buildTransitionTable(self, name):

        print('Building a transition table for \"%s\" ...' % name)
        prog = progressbar.ProgressBar(widgets=[progressbar.Percentage(), progressbar.Bar()], maxval=self.n_actions_objs[name])
        prog.start()

        if self.use_multiproc:
            actions_all = []
            for direction in self.actions_objs[name]:
                for actions in self.actions_objs[name][direction]:
                    actions_all = actions_all + actions

            ret = self.pool.map(computeTransition_multiproc, 
                                [(name,
                                  self.name2cells[name],
                                  action,
                                  self.name_actor,
                                  self.type_actor,
                                  self.dims_actor) for action in actions_all])

            for i, action in enumerate(actions_all):
                if ret[i] is None:
                    continue
                key = self.action2key(action)
                self.trans_objs[name][key] = ret[i]

            prog.update(len(self.trans_objs[name])-1)
        else:
            for direction in self.actions_objs[name]:
                for actions in self.actions_objs[name][direction]:
                    for action in actions:
                        key = self.action2key(action)
                        tran = self.computeTransition(name, action)
                        if tran is not None:
                            self.trans_objs[name][key] = tran
                        
                        prog.update(len(self.trans_objs[name])-1)
        prog.finish()

    def lookupTransitionTable(self, name, x, y, yaw, action):

        key = self.action2key(action)
        if key in self.trans_objs[name]:
            tran = self.trans_objs[name][key]
        else:
            tran = self.computeTransition(name, action)
            if tran is not None:
                self.trans_objs[name][key] = tran
            else:
                cellpos = self.getCellPos(name,x,y,yaw)
                return (x,y,yaw,cellpos)

        vec = tran['vec']
        x_next = np.cos(yaw)*vec[0]-np.sin(yaw)*vec[1] + x
        y_next = np.sin(yaw)*vec[0]+np.cos(yaw)*vec[1] + y
        yaw_next = yaw + tran['yaw']
        cellpos_next = self.getCellPos(name,x_next,y_next,yaw_next)

        return (x_next,y_next,yaw_next,cellpos_next)

    def costAction(self, action_prv, action):

        for key, value in action.items() if type(action)==dict else action:
            if key=='pos':
                pos = value
            elif key=='velocity':
                velocity = value
            elif key=='duration':
                duration = value

        #cost = np.linalg.norm(velocity) * duration
        cost = 0.1
        if action_prv is None:
            return cost
        
        for key, value in action_prv.items() if type(action_prv)==dict else action_prv:
            if key=='pos':
                pos_prv = value
                break

        length = np.linalg.norm(np.array(pos)-np.array(pos_prv))
        return 0.1 + length + 0.1 + cost

    def centerOfMass(self, name, x, y, yaw):

        cx = np.cos(yaw)*self.com_objs[name][0]-np.sin(yaw)*self.com_objs[name][1] + x
        cy = np.sin(yaw)*self.com_objs[name][0]+np.cos(yaw)*self.com_objs[name][1] + y

        return (cx,cy)

    def isStateValid(self, state):

        x   = state.getX()
        y   = state.getY()
        yaw = state.getYaw()

        # [KUKA]
        #dist = np.linalg.norm([x,y])
        #if dist < 0.25 or dist > 0.80:
        #    return False

        cx,cy = self.centerOfMass(self.name_target, x,y,yaw)
        dist = np.linalg.norm([cx-0.63,cy])
        if dist > self.setup['table_radius'] + 0.25:
            return False

        self.sim_cols[self.name_target].setObjectPose(self.name_target, x, y, yaw)
        if self.sim_cols[self.name_target].isObjectCollide(self.name_target)==True:
            return False
        return True

    def isObjectStable(self, name, x,y,yaw):

        if (x,y,yaw) in self.stbs_objs[name]:
            return self.stbs_objs[name][(x,y,yaw)]

        cx,cy = self.centerOfMass(name, x,y,yaw)
        dist = np.linalg.norm([cx-0.63,cy])
        if dist < 0.40:
            return True

        self.sim_stbs[name].setObjectPose(name,x,y,yaw)
        (x0,y0,z0),q0 = self.sim_stbs[name].getObjectPose(name,ndims=3)
        for i in range(100):
            self.sim_stbs[name].stepSimulation()
        (x1,y1,z1),q1 = self.sim_stbs[name].getObjectPose(name,ndims=3)

        q0 = q0.normalised
        q1 = q1.normalised
        dist_q = 1 - np.dot(q0.q,q1.q)**2

        ret = z1>0 or dist_q<0.003 # <10degree
        self.stbs_objs[name][(x,y,yaw)] = ret

        return ret

    def isActionStable(self, name, x,y,yaw, action):

        x_next, y_next, yaw_next, cellpos_next\
         = self.lookupTransitionTable(name,x,y,yaw,action)
        if np.linalg.norm([x     -0.63,y     ]) < 0.40 and\
           np.linalg.norm([x_next-0.63,y_next]) < 0.40:
            return True

        self.sim_stbs[name].setObjectPose(name,x,y,yaw)
        (x0,y0,z0),q0 = self.sim_stbs[name].getObjectPose(name,ndims=3)
        self.applyAction(self.sim_stbs[name], \
                         name,x,y,yaw, \
                         action,self.name_actor,self.dims_actor)
        (x1,y1,z1),q1 = self.sim_stbs[name].getObjectPose(name,ndims=3)

        q0 = q0.normalised
        q1 = q1.normalised
        dist_q = 1 - np.dot(q0.q,q1.q)**2

        return z1>0 or dist_q<0.003 # <10degree

    def selectGoals(self, name, goals):

        names = self.name2names[name]
        probs = self.name2probs[name]
        goals_ret = []
        for goal in goals:
            prob = 0
            for i in range(len(names)):
                if self.isObjectStable(names[i], goal[0], goal[1], goal[2]):
                    prob = prob + probs[i]
            
            if prob > 0.90:
                goals_ret.append(goal)
        return goals_ret

    @staticmethod
    def split( path, xy_min=0.05, yaw_min=15.0/180.0*np.pi, method='rotNpushNrot' ):

        res = [path[0]]        
        for i in range(1,len(path)):

            x0,y0,yaw0 = path[i-1]
            x1,y1,yaw1 = path[i]

            if method=='smooth':
                vec_xy = np.array([x1-x0,y1-y0])
                vec_yaw = distance_angle(yaw1,yaw0)
                dist_xy  = np.linalg.norm(vec_xy)
                dist_yaw = abs(vec_yaw)
                n_xy  = int(np.ceil(dist_xy  / xy_min))
                n_yaw = int(np.ceil(dist_yaw / yaw_min))
                n_split = n_xy if n_xy > n_yaw else n_yaw
                vec_xy = vec_xy / n_split
                vec_yaw = vec_yaw / n_split
                for i in range(n_split):
                    res.append([x0+vec_xy[0]*(i+1), 
                                y0+vec_xy[1]*(i+1), 
                                yaw0+vec_yaw*(i+1)])
            elif method=='rotNpushNrot':

                vec_xy = np.array([x1-x0,y1-y0])
                dist_xy  = np.linalg.norm(vec_xy)
                yaw01 = math.acos(vec_xy[1]/dist_xy)

                vec_yaw = distance_angle(yaw01,yaw0)
                dist_yaw = abs(vec_yaw)
                n_yaw = int(np.ceil(dist_yaw / yaw_min))
                if n_yaw > 0:
                    vec_yaw = vec_yaw / n_yaw
                    for i in range(n_yaw):
                        res.append([x0,y0,yaw0+vec_yaw*(i+1)])

                vec_xy = np.array([x1-x0,y1-y0])
                dist_xy  = np.linalg.norm(vec_xy)
                n_xy  = int(np.ceil(dist_xy  / xy_min))
                if n_xy > 0:
                    vec_xy = vec_xy / n_xy
                    for i in range(n_xy):
                        res.append([x0+vec_xy[0]*(i+1),
                                    y0+vec_xy[1]*(i+1), yaw01])

                vec_yaw = distance_angle(yaw1,yaw01)
                dist_yaw = abs(vec_yaw)
                n_yaw = int(np.ceil(dist_yaw / yaw_min))
                if n_yaw > 0:
                    vec_yaw = vec_yaw / n_yaw
                    for i in range(n_yaw):
                        res.append([x1,y1,yaw01+vec_yaw*(i+1)])

        return res

    def plan_rrt(self, name, init, goal):

        init = self.createState(*tuple(init))
        goal = self.createState(*tuple(goal))

        self.name_target = name

        planner = og.RRTstar(self.si)
        ss = og.SimpleSetup(self.si)
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid))    
        ss.setPlanner(planner)
        ss.setStartAndGoalStates(init, goal)        
        solved = ss.solve(1.0)
        if solved:
            ss.simplifySolution()
            path = ss.getSolutionPath()

            path_res = []
            for state in path.getStates():
                path_res.append((state.getX(),state.getY(),state.getYaw()))
            return path_res
        else:
            return None

    def plan_guided(self, name, path_guide, pred_step=np.inf):

        def findBestAction(name, x, y, yaw, cellpos_goal):
            err_min = -1
            for action in self.trans_objs[name]:
                x_next, y_next, yaw_next, cellpos_next\
                 = self.lookupTransitionTable(name,x,y,yaw,action)
                err = CellPhysicsBase.distanceCellPos(cellpos_next,cellpos_goal)
                if err_min==-1 or err_min > err:
                    err_min = err
                    ret = (action, err, x_next, y_next, yaw_next, cellpos_next)
            return ret

        waypoints = self.split(path_guide)
        path_res = [path_guide[0]]
        actions_res = []

        i_cur  = 0
        x_cur, y_cur, yaw_cur = path_guide[0]
        while i_cur < len(waypoints)-1:
            i_min = -1
            err_min = -1
            for i_next in range(min(i_cur+5,len(waypoints)-1),i_cur,-1):

                x,y,yaw = waypoints[i_next]                
                cellpos = self.getCellPos(name,x,y,yaw)
                action_next, err, x_next, y_next, yaw_next, cellpos_next\
                 = findBestAction(name,x_cur,y_cur,yaw_cur,cellpos)

                if err_min==-1 or err_min > err:
                    err_min = err
                    i_min = i_next                        
                    next_min = (action_next,x_next,y_next,yaw_next,cellpos_next)
                    cellpos_guide = cellpos

            i_cur = i_min
            action_cur,x_cur,y_cur,yaw_cur,cellpos_next = next_min
            path_res.append([x_cur,y_cur,yaw_cur])
            actions_res.append(action_cur)

            if len(actions_res) >= pred_step:
                break

        return path_res, actions_res

    def plan_bfs(self, name, waypoints, horizon=1, thres=0.03):

        if horizon==0:
            return [waypoints[0]], [], 0, 0

        if len(self.trans_objs[name])==0:
            self.buildTransitionTable(obj)

        def get_best_action(x,y,yaw,cellpos_goal,path,actions,cost,depth):

            if depth==0:
                self.sim_objs[name].setObjectPose(name,x,y,yaw)
                cellpos = self.sim_objs[name].getCellBodyPose(name)
                err = CellPhysicsBase.distanceCellPos(cellpos,cellpos_goal)
                return (err,list(path),list(actions),cost)

            actions_sorted = []
            action_prv = actions[-1] if len(actions)>0 else None
            for action in self.trans_objs[name]:
                cost_cur = self.costAction(action_prv,action)
                actions_sorted.append((action,cost_cur))
            actions_sorted = sorted(actions_sorted, key=lambda a: a[1])

            err_min = -1
            path_min = None
            actions_min = None
            cost_min = None
            for action, cost_cur in actions_sorted:

                x_next,y_next,yaw_next,cellpos_next\
                 = self.lookupTransitionTable(name,x,y,yaw,action)
                err_next = CellPhysicsBase.distanceCellPos(cellpos_next,cellpos_goal)
                
                path_next    = list(path)    + [(x_next,y_next,yaw_next)]
                actions_next = list(actions) + [action]
                cost_next    = cost + cost_cur
                
                if err_next < thres:
                    err_res, path_res, actions_res, cost_res\
                     = (err_next, path_next, actions_next, cost_next)
                else:
                    err_res, path_res, actions_res, cost_res\
                     = get_best_action( x_next,y_next,yaw_next,cellpos_goal,
                                        path_next, actions_next, cost_next, depth-1 )
                    if err_res==-1:
                        continue
                
                if err_res < thres:
                    return (err_res, path_res, actions_res, cost_res)
                elif err_min==-1 or err_min > err_res:
                    err_min = err_res
                    path_min = path_res
                    actions_min = actions_res
                    cost_min = cost_res

            return (err_min, path_min, actions_min, cost_min)

        x0,y0,yaw0 = waypoints[0]
        x1,y1,yaw1 = waypoints[1]
        cellpos0 = self.getCellPos(name,x0,y0,yaw0)
        cellpos1 = self.getCellPos(name,x1,y1,yaw1)
        path = [waypoints[0]]
        actions = []
        cost = 0
        i = 1
        err_prv = np.inf
        while True:
            err_res, path_res, actions_res, cost_res\
             = get_best_action(x0,y0,yaw0,cellpos1,[],[],0,horizon)

            if self.debug:
                for action in actions_res:
                    self.applyAction(self.sim_debug,name,x0,y0,yaw0,action,self.name_actor,self.dims_actor)
                    (x0,y0,yaw0) = self.sim_debug.getObjectPose(name)
                    actions.append(action)
                    path.append((x0,y0,yaw0))
                cost = cost + cost_res                
            else:
                x0,y0,yaw0 = path_res[-1]                    
                actions = actions + actions_res
                path = path + path_res
                cost = cost + cost_res

            if len(actions) >= horizon:
                break

            cellpos0 = self.getCellPos(name,x0,y0,yaw0)
            if err_res < thres:
                i = i + 1
                if i<len(waypoints):
                    x1,y1,yaw1 = waypoints[i]
                    cellpos1 = self.getCellPos(name,x1,y1,yaw1)
                else:
                    break
            elif i>=len(waypoints)-1:
                if err_prv < err_res:
                    break
                else:
                    err_prv = err_res

        return path, actions

    def plan_diff(self, name, waypoints, horizon=1, thres=0.03, pred_step=np.inf):

        path = [waypoints[0]]
        actions = []
        for i in range(1,len(waypoints)):

            x0,y0,yaw0 = waypoints[i-1]
            x1,y1,yaw1 = waypoints[i]
            cellpos0 = self.getCellPos(name,x0,y0,yaw0)
            cellpos1 = self.getCellPos(name,x1,y1,yaw1)

            if i==len(waypoints)-1:
                thres = 0.01

            err = np.inf
            while err > thres:

                # cached simulation result
                errs = []
                for action in self.trans_objs[name]:
                    for key, value in action:
                        if key=='direction':
                            direction = value
                        elif key=='idx':
                            idx = value
                        elif key=='ldx':
                            ldx = value

                    (x_cur,y_cur,yaw_cur,cellpos_cur)\
                     = self.lookupTransitionTable(name,x0,y0,yaw0,action)
                    err = CellPhysicsBase.distanceCellPos(cellpos_cur,cellpos1)
                    errs.append((err,(direction,idx,ldx),(x_cur,y_cur,yaw_cur)))

                if len(errs)>0:
                    err_min = min(errs,key=lambda x: x[0])
                    errs = [err_min]

                # fix the pushing length (ldx)
                dist = np.sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0))
                for ldx, length in enumerate(self.pushing_dists):
                    if dist < length:
                        break

                vec_com2goal = np.array(self.centerOfMass(name,x1,y1,yaw1))\
                                - np.array(self.centerOfMass(name,x0,y0,yaw0))
                np.array(self.centerOfMass(name,x1,y1,yaw1))
                for direction in self.actions_objs[name]:

                    # ignore the opposite direction
                    vec = [0,0]
                    vec[0] = np.cos(yaw0)*direction[0]-np.cos(yaw0)*direction[1]
                    vec[1] = np.sin(yaw0)*direction[0]+np.cos(yaw0)*direction[1]
                    if vec_com2goal.dot(np.array(vec)) < 0:
                        continue

                    # start searching from the point that is closest to the line between center of mass                    
                    uvec_c2c = self.uvec_contact2com[name][direction]
                    vec = np.zeros(uvec_c2c.shape)
                    vec[:,0] = np.cos(yaw0)*uvec_c2c[:,0]-np.sin(yaw0)*uvec_c2c[:,1]
                    vec[:,1] = np.sin(yaw0)*uvec_c2c[:,0]+np.cos(yaw0)*uvec_c2c[:,1]

                    projs = vec.dot(vec_com2goal)
                    idx = np.argmax(projs)

                    action = self.actions_objs[name][direction][idx][ldx]
                    (x_cur,y_cur,yaw_cur,cellpos_cur)\
                     = self.lookupTransitionTable(name,x0,y0,yaw0,action)
                    err_cur = CellPhysicsBase.distanceCellPos(cellpos_cur,cellpos1)
                    errs.append((err_cur,(direction,idx,ldx),(x_cur,y_cur,yaw_cur)))

                    for incr in [-1,1]:
                        j = idx + incr
                        while 0<=j and j<len(self.actions_objs[name][direction]):
                            actioni = self.actions_objs[name][direction][j][ldx]
                            (xi,yi,yawi,cellposi)\
                             = self.lookupTransitionTable(name,x0,y0,yaw0,actioni)
                            erri = CellPhysicsBase.distanceCellPos(cellposi,cellpos1)
                            if erri > err_cur:
                                break
                            
                            errs.append((erri,(direction,j,ldx),(xi,yi,yawi)))
                            err_cur = erri
                            j = j + incr

                # fix the contact point (direction,idx)
                err_min = min(errs,key=lambda x: x[0])
                (err,(direction,idx,ldx),(x,y,yaw)) = err_min

                errs = [err_min]
                err_cur = err
                for incr in [-1,1]:
                    l = ldx + incr
                    while 0<=l and l<len(self.pushing_dists):
                        actionl = self.actions_objs[name][direction][idx][l]
                        (xl,yl,yawl,cellposl)\
                         = self.lookupTransitionTable(name,x0,y0,yaw0,actionl)
                        errl = CellPhysicsBase.distanceCellPos(cellposl,cellpos1)
                        if errl > err_cur:
                            break

                        errs.append((errl,(direction,idx,l),(xl,yl,yawl)))
                        err_cur = errl
                        l = l + incr
                
                err_min = min(errs,key=lambda x: x[0])
                (err,(direction,idx,ldx),(x,y,yaw)) = err_min

                action = self.actions_objs[name][direction][idx][ldx]
                self.applyAction(self.sim,name,x0,y0,yaw0,action,self.name_actor,self.dims_actor)
                (x,y,yaw) = self.sim.getObjectPose(name)

                actions.append(action)
                path.append((x,y,yaw))
                (x0,y0,yaw0) = (x,y,yaw)
                cellpos0 = self.getCellPos(name,x0,y0,yaw0)

                print('# of simulation: {}/{}, err: {}'.format(len(self.trans_objs[name]),self.n_actions_objs[name],err))

                if len(actions) >= pred_step:
                    break
            if len(actions) >= pred_step:
                break

        return path, actions

    def plan_prop(self, name, waypoints, horizon=1, thres=0.03, method='bfs', n_plans=10, actions_rejected=[]):

        for name_i in self.name2names[name]:
            if len(self.trans_objs[name_i])==0:
                self.buildTransitionTable(name_i)

        keys_rejected = []
        for action in actions_rejected:
            keys_rejected.append(self.action2key(action))

        x0,y0,yaw0 = waypoints[0]
        x1,y1,yaw1 = waypoints[1]
        cellpos0 = self.getCellPos(name+'_0',x0,y0,yaw0)
        cellpos1 = self.getCellPos(name+'_0',x1,y1,yaw1)

        actions = [action for direction in self.actions_objs[name+'_0']\
                            for actions in self.actions_objs[name+'_0'][direction]\
                              for action in actions                           ]

        plan_all = []
        for action in actions:
            if self.action2key(action) in keys_rejected:
                continue

            cost = self.costAction(None,action)
            probs = self.name2probs[name]
            err = 0
            nexts = []
            for i, name_i in enumerate(self.name2names[name]):
                x_i,y_i,yaw_i,cellpos_i\
                 = self.lookupTransitionTable(name_i,x0,y0,yaw0,action)
                err_i = CellPhysicsBase.distanceCellPos(cellpos_i,cellpos1)
                err = err + err_i*probs[i]
                nexts.append((name_i,x_i,y_i,yaw_i,probs[i]))
            nexts = sorted(nexts, key=lambda x: -x[-1])
            plan_all.append({'action': action, 'error': err, 'nexts':nexts})

        plan_all = sorted(plan_all, key=lambda x: x['error'])

        plan_top = []
        for plan in plan_all:
            succ = 0
            if self.use_multiproc:
                ret = self.pool.map(isActionStable_multiproc, 
                                    [(name_i,
                                      self.name2cells[name_i],
                                      x_i, y_i, yaw_i,
                                      plan['action'],
                                      self.name_actor,
                                      self.type_actor,
                                      self.dims_actor) 
                                      for name_i,x_i,y_i,yaw_i,_ in plan['nexts']])

                for i, is_stable in enumerate(ret):
                    if is_stable:
                        prob_i = plan['nexts'][i][-1]
                        succ = succ + prob_i
            else:
                for i, nexts in enumerate(plan['nexts']):
                    name_i, x_i, y_i, yaw_i, prob_i = nexts
                    if self.isActionStable(name_i,x0,y0,yaw0, plan['action']):
                        succ = succ + prob_i

            plan['succ'] = succ
            plan_top.append(plan)
            if len(plan_top)>=n_plans:
                break

        return plan_top

    def draw_cellpos(self, ax, cellpos, color=[0,0,0], marker='s'):
        cellpos = np.array(cellpos)
        ax.plot(-cellpos[:,2],cellpos[:,1],'.',marker=marker,color=color)

    def draw_action(self, ax, x,y,yaw, action, color='g', marker='o'):
        for key, value in action.items() if type(action)==dict else action:
            if key=='pos':
                pos = value
            elif key=='velocity':
                velocity = value
        tmp = [0,0]
        tmp[0] = np.cos(yaw)*pos[0]-np.sin(yaw)*pos[1] + x
        tmp[1] = np.sin(yaw)*pos[0]+np.cos(yaw)*pos[1] + y
        ax.plot(-tmp[1],tmp[0],'.',color=color,marker=marker)

        tmp2 = [0,0]
        tmp2[0] = np.cos(yaw)*velocity[0]-np.sin(yaw)*velocity[1] + tmp[0]
        tmp2[1] = np.sin(yaw)*velocity[0]+np.cos(yaw)*velocity[1] + tmp[1]
        ax.plot([-tmp[1],-tmp2[1]],[tmp[0],tmp2[0]],'-',color=color,marker=marker)

    def draw_plan(self, ax, name, path, actions):

        circle = plt.Circle((0,0.63),0.5, color='k', fill=False)
        ax.add_artist(circle)
        clrs = np.linspace(0.3,1,len(path)+1)
        for i in range(len(path)-1):
            cellpos = self.getCellPos(name,path[i][0],path[i][1],path[i][2])
            self.draw_cellpos(ax,cellpos,color=(clrs[i+1],0,0))
            action = actions[i]
            self.draw_action(ax,path[i][0],path[i][1],path[i][2], action )
        cellpos = self.getCellPos(name,path[-1][0],path[-1][1],path[-1][2])
        self.draw_cellpos(ax,cellpos,color=(clrs[-1],0,0))
        ax.set_aspect('equal')

    def plan(self, name, init, goal, method='diff', pred_step=np.inf, thres=0.03, n_plans=10, waypoints=None, actions_rejected=[]):

        if waypoints is None:
            if method=='prop':
                waypoints = self.plan_rrt(name+'_0', init, goal)
            else:
                waypoints = self.plan_rrt(name, init, goal)

        #print('waypoints: ',waypoints)
        if method=='guided':
            path_res, actions_res = self.plan_guided(name, waypoints, horizon=pred_step, thres=thres)
        elif method=='bfs':
            path_res, actions_res = self.plan_bfs(name, waypoints, horizon=pred_step, thres=thres)
        elif method=='diff':
            path_res, actions_res = self.plan_diff(name, waypoints, horizon=pred_step, thres=thres)
        elif method=='prop':
            plans = self.plan_prop(name, waypoints, horizon=1, thres=thres, n_plans=n_plans, actions_rejected=actions_rejected)
            for plan in plans:
                if plan['succ'] >= 1.00:
                    return [init, plan['nexts'][0][1:4]], [plan['action']]
            plan_max = max(plans, key=lambda x: -x['succ'])
            return [init, plan_max['nexts'][0][:3]], [plan_max['action']]

        if self.debug:

            fig = plt.figure()
            ax = fig.add_subplot(111)
            self.draw_plan(ax, name, path_res, actions_res)
            plt.show()
            fig.savefig(os.path.join('./',method+'.png'))

            self.sim_debug.recordStart(os.path.join('./',method+'.mp4'))
            self.sim_debug.setupCamera(0.75,-90,-89.99, 0.63,0,0)
            x,y,yaw = init
            for action in actions_res:
                self.applyAction(self.sim_debug,name,x,y,yaw,action,self.name_actor,self.dims_actor)
                x,y,yaw = self.sim_debug.getObjectPose(name)
            self.sim_debug.recordStop()

        return (path_res, actions_res)

if __name__=='__main__':

    obj = 'hammer'
    obj2mpp = {'hammer':0.35/500.0}

    planner = KinoPlanner(0.02,[0.10,0.05,0.03,0.02],debug=True, use_multiproc=True)
    fp_label = '../../dataset/objects/%s_label.pgm' % obj
    planner.addObject(obj,fp_label, [1,1,1,1], meter_per_pixel=obj2mpp[obj])

    time_beg = time.time()
    path, actions = planner.plan(obj,[0.63,0,0],[0.25,0.00,-90.0/180.0*np.pi], method='prop')
    time_end = time.time()
    print('time: {}'.format(time_end - time_beg))
    print('# of actions: {}'.format(len(actions)))
