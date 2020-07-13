import os, sys
dir_software = os.path.join('..')
if os.path.abspath(dir_software) not in sys.path:
    sys.path.append(dir_software)

from cellphysics import DiffPhysics, params
from kinoplanner import KinoPlanner
from simulator import distance_angle
from simulator import SimBullet

import numpy as np
import cv2
import matplotlib
import matplotlib.pyplot as plt

class Pose2D:
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

class CellPhysicsPlanner:

    def __init__(self, name, img, masses, meter_per_pixel, n_iters=10):

        self.name = name
        self.n_iters = n_iters

        if type(img)==str:
            img = cv2.imread(img,cv2.IMREAD_UNCHANGED)

        self.estimator = DiffPhysics(is_dbg=False)
        setups = {'actor':{'name':'finger','type':'finger'},
                  'target_infos':[{'name':name,'label':img,'masses':masses,
                                   'meter_per_pixel':meter_per_pixel}],
                  'mass_minmax':[0.1,5],
                  'train':[],
                  'test':[],
                  'real':[]}
        self.estimator.setup(setups, n_group=-1)
        self.planner = KinoPlanner(debug=False, contact_resolution=0.01, pushing_dists=[0.20,0.15,0.10,0.05,0.03,0.02,0.01])
        self.planner.addObject(name,img,masses,meter_per_pixel)
        
        cellinfos = []
        for mass in masses:
            cellinfos.append({'mass':mass})
        self.history = [{'cellinfos':cellinfos}]

    def addObservation(self, pos_begin, pos_final, pos_push, vec_push, duration):

        x_tmp = pos_final.x - pos_begin.x
        y_tmp = pos_final.y - pos_begin.y
        x_res = np.cos(-pos_begin.theta)*x_tmp - np.sin(-pos_begin.theta)*y_tmp
        y_res = np.sin(-pos_begin.theta)*x_tmp + np.cos(-pos_begin.theta)*y_tmp
        yaw_res = distance_angle(pos_final.theta, pos_begin.theta)
        self.estimator.sim.setObjectPose(self.name,x_res,y_res,yaw_res)
        cellpos_res = self.estimator.sim.getCellBodyPose(self.name)

        pos = [0,0]
        x_tmp = pos_push.x - pos_begin.x
        y_tmp = pos_push.y - pos_begin.y
        pos[0] = np.cos(-pos_begin.theta)*x_tmp - np.sin(-pos_begin.theta)*y_tmp
        pos[1] = np.sin(-pos_begin.theta)*x_tmp + np.cos(-pos_begin.theta)*y_tmp

        velocity = [0,0,0]
        vel_x = vec_push.x / duration
        vel_y = vec_push.y / duration
        velocity[0] = np.cos(-pos_begin.theta)*vel_x-np.sin(-pos_begin.theta)*vel_y
        velocity[1] = np.sin(-pos_begin.theta)*vel_x+np.cos(-pos_begin.theta)*vel_y

        action = {'finger':{'pos':pos,'yaw':0,'velocity':velocity,'duration':duration},
                  'targets':[self.name], 
                  self.name:{'pos':[0,0],'yaw':0,
                             'pos_res':(x_res,y_res),'yaw_res':yaw_res}}

        self.estimator.actions_train.append(action)
        self.estimator.poses_refs.append({})
        self.estimator.poses_refs[-1][self.name]\
         = {'x':x_res,'y':y_res,'yaw':yaw_res,'cellpos':cellpos_res}

        if self.estimator.cellinfos_infer is None:
            cellinfos_init = None
        else:
            cellinfos_init = self.estimator.cellinfos_infer[self.name]
        res = self.estimator.infer(errlimit=0.01, n_iters=10,
                                   cellinfos_init=cellinfos_init)

        cellinfos = self.estimator.cellinfos_infer[self.name]
        self.planner.updateObjectProperties(self.name,cellinfos)

        cellinfos_copy = [info.copy() for info in cellinfos]
        self.history.append({'x0'  :pos_begin.x,
                             'y0'  :pos_begin.y,
                             'yaw0':pos_begin.theta,
                             'x1'  :pos_final.x,
                             'y1'  :pos_final.y,
                             'yaw1':pos_final.theta,
                             'pos_push':(pos_push.x,pos_push.y),
                             'vec_push':(vec_push.x,vec_push.y),
                             'duration':duration,
                             'cellinfos':cellinfos_copy,
                             'train_err_cell':res[-1]['error_cell']})

    def plan(self, init, goal, pred_step=1):
        
        path, actions = self.planner.plan(self.name,init,goal,pred_step=pred_step)
        return path, actions

    def getProperties(self, idx=-1):
        return self.history[idx]['cellinfos']

    def drawProperties(self, cellinfos=None):

        self.estimator.sim.setObjectPose('finger',10,0,0)
        if cellinfos is None:
            cellinfos = self.estimator.cellinfos_infer[self.name]

        cnorm = matplotlib.colors.Normalize(vmin=0.1,vmax=5)
        for info in cellinfos:
            info['color'] = matplotlib.cm.coolwarm(cnorm(info['mass']))
        self.estimator.sim.setCellBodyProperties(self.name,cellinfos)
        self.estimator.sim.setObjectPose(self.name,0,0,0)
        self.estimator.sim.setupCamera(0.50,-90,-89.99)
        img_prop = self.estimator.sim.draw()
        img_prop = cv2.cvtColor(img_prop, cv2.COLOR_RGB2BGR)    
        return img_prop

    def drawPlan(self, path, actions):

        fig = plt.figure(figsize=[4,3])
        ax = fig.add_subplot(111)
        self.planner.draw_plan(ax, self.name, path, actions)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_xlim([-0.80, 0.80])
        ax.set_ylim([ 0.13, 1.13])
        fig.tight_layout()
        fig.canvas.draw() 
        w,h = fig.canvas.get_width_height()
        img_plan = np.fromstring(fig.canvas.tostring_argb(),dtype=np.uint8)
        img_plan.shape = (h,w,4)
        img_plan = np.roll(img_plan,3,axis=2)
        return img_plan

if __name__=='__main__':

    obj = 'hammer'
    init = [0.63,0.00,0.5*np.pi]
    goal = [0.40,-0.40,-30.0/180.0*np.pi]
    meter_per_pixel = 0.53/640.0

    fp_label = '../../dataset/objects/'+obj+'_label.pgm'
    img = cv2.imread(fp_label, cv2.IMREAD_UNCHANGED)

    planner = CellPhysicsPlanner(obj, img, [1,1,1,1], meter_per_pixel)

    sim = SimBullet(gui=True)
    sim.setupCamera(0.75,-90,-89.99, 0.63,0,0)
    sim.addObject('table_round','table_round',[1,1,1,1])
    sim.setObjectPose('table_round',0.63,0,0)
    sim.addObject('finger','finger',[1,1,0,1])
    sim.setObjectPose('finger',10,0,0)
    cells = DiffPhysics.createCells(img,params['hammer'][6]['masses'],meter_per_pixel=meter_per_pixel)
    sim.addObject(obj, cells, [0.5,0.5,0.5,1], 0, 0, 0)
    sim.setObjectPose(obj,goal[0],goal[1],goal[2])
    cellpos_goal = sim.getCellBodyPose(obj)
    sim.setObjectPose(obj,init[0],init[1],init[2])

    class Pose2D:
        def __init__(self,x,y,theta):
            self.x = x
            self.y = y
            self.theta = theta

    history = {'path':[], 'actions':[]}
    while True:
        x0,y0,yaw0 = sim.getObjectPose(obj)
        #cellpos_curr = sim.getCellBodyPose(obj)
        #err_cell = DiffPhysics.distanceCellPos(cellpos_goal, cellpos_curr)
        #if err < 0.01:
        #    break
        err_xy = np.linalg.norm([goal[0]-x0,goal[1]-y0])
        err_yaw = distance_angle(goal[2],yaw0)
        cellinfos = planner.getProperties()
        cellinfos_copy = [info.copy() for info in cellinfos]
        history['path'].append({'x':x0,'y':y0,'yaw':yaw0,'cellinfos':cellinfos_copy})

        print('err_xy: %.3f err_yaw: %.3f' %(err_xy,err_yaw))
        if err_xy <= 0.02 and err_yaw/np.pi*180.0 <= 15:
            print('[Success]')
            break

        path, actions = planner.plan([x0,y0,yaw0], goal)
        action = actions[0]
        history['actions'].append(action.copy())

        planner.planner.applyAction(sim,obj,x0,y0,yaw0,action)
        x1,y1,yaw1 = sim.getObjectPose(obj)
        sim.setObjectPose('finger',10,0,0)

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
        
        cellinfos = planner.getProperties()
        cellinfos_sim = sim.getCellBodyProperties(obj)
        cnorm = matplotlib.colors.Normalize(vmin=0.1,vmax=5)
        for info, info_sim in zip(cellinfos,cellinfos_sim):
            info_sim['color'] = matplotlib.cm.coolwarm(cnorm(info['mass']))
        sim.setCellBodyProperties(obj,cellinfos_sim)

        x0,y0,yaw0 = (x1,y1,yaw1)
