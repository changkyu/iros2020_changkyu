import os, sys
import time
from sklearn.cluster import KMeans

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import cv2

dir_root = os.path.abspath('..')
if os.path.abspath(dir_root) not in sys.path:
    sys.path.append(dir_root)
from simulator import SimBullet

class CellPhysicsBase:

    def __init__(self, Simulator=SimBullet, is_dbg=False, fp_video=''):
        self.is_dbg = is_dbg
        self.is_rec = len(fp_video)>0
        self.is_vis = self.is_dbg or self.is_rec
        self.fp_video = fp_video

        self.n_sims = 0
        self.time_sims = 0
        self.meta_objects = []
        self.sim = Simulator(gui=self.is_vis)
        self.sim.addObject('table_rect','table_rect',[0,0,0,1])

        self.cellinfos_infer = None

        if self.is_vis:
            #self.sim.setupCamera(0.75,-90,-60)
            self.sim.setupCamera(0.30,-90,-89.99)

        self.debugId = -1
    
    @staticmethod
    def distanceCellPos(cellpos_A,cellpos_B):

        dist = 0
        for pos_A, pos_B in zip(cellpos_A, cellpos_B):
            #dist = dist + np.linalg.norm(np.array(pos_A)-np.array(pos_B))
            dist = dist + np.sqrt((pos_A[1]-pos_B[1])**2+(pos_A[2]-pos_B[2])**2)
        dist = dist / float(len(cellpos_A))
        return dist

    @staticmethod
    def createCells(img, masses, meter_per_pixel, n_group=-1):

        if type(img)==str:
            img = cv2.imread(img, cv2.IMREAD_UNCHANGED)
        labels = np.unique(img)
        
        np.sort(labels)
        if labels[0]==0:
            labels = labels[1:]

        for w_resize in [50,40,30,20,10]:

            scale = img.shape[1]*meter_per_pixel
            
            cells = []
            group = []
            cell2group = []
            group2cell = {}
            cx = 0
            cy = 0
            siz = (w_resize,int(img.shape[0]*w_resize/img.shape[1]))
            for i, label in enumerate(labels):
                if label==0:
                    continue

                group.append(label)
                mass = masses[i]
                occ = img==label
                occ_resize = cv2.resize(occ.astype(np.uint8),siz,\
                                        interpolation=cv2.INTER_NEAREST)
                #idxes_cell = []
                if (occ_resize.sum(axis=0)>0).sum() >=2 and \
                   (occ_resize.sum(axis=1)>0).sum() >=2:
                    for r in range(occ_resize.shape[0]):
                        for c in range(occ_resize.shape[1]):
                            if occ_resize[r,c]:
                                x = -r*scale/float(w_resize)
                                y = -c*scale/float(w_resize)
                                cx = cx + x
                                cy = cy + y
                                cells.append({'pos':[x,y],'mass':mass})
                                #cell2group.append(label)
                                #idxes_cell.append(len(cells)-1)

                #group2cell[label] = idxes_cell

            cx = cx / float(len(cells))
            cy = cy / float(len(cells))
            for cell in cells:
                cell['pos'][0] = cell['pos'][0] - cx
                cell['pos'][1] = cell['pos'][1] - cy

            idx_ctr = -1
            dist_ctr = -1
            for c, cell in enumerate(cells):
                dist = cell['pos'][0]**2 + cell['pos'][1]**2
                if dist_ctr==-1 or dist < dist_ctr:
                    dist_ctr = dist
                    idx_ctr = c

            if len(cells)>127:                
                continue

            if n_group>=0:
                positions = np.argwhere(img>0)
                kmeans = KMeans(n_group).fit(positions)

                img_group = np.zeros(img.shape,dtype=int)
                for i, pos in enumerate(positions):
                    img_group[pos[0],pos[1]] = kmeans.labels_[i]+1

                group2label = []
                for r,c in kmeans.cluster_centers_:
                    r,c = int(r),int(c)
                    group2label.append(img[r,c])

                for i, cell in enumerate(cells):
                    r = int(-(cell['pos'][0] + cx)/float(meter_per_pixel))
                    c = int(-(cell['pos'][1] + cy)/float(meter_per_pixel))

                    g = int(img_group[r,c]-1)
                    cell2group.append(g)
                    if g not in group2cell:
                        group2cell[g] = [i]
                    else:
                        group2cell[g].append(i)
            else:
                for i, cell in enumerate(cells):
                    cell2group.append(i)
                    group2cell[i] = [i]
            break

        return {'cells':cells,
                'cell_size':scale/float(w_resize)*0.5,
                'idx_center':idx_ctr,
                'group':group,
                'cell2group':cell2group, 'group2cell':group2cell,
                'offset':[-cx,-cy], 'meter_per_pixel':meter_per_pixel}

    @staticmethod
    def simulate(sim, action, name_actor, dims_actor, 
                      cnormM=lambda x: x, cnormf=lambda x: x, 
                      traj_names=None, traj_ref=None,
                      is_dbg=False, is_vis=False              ):

        #time_start = time.time()
        #self.n_sims = self.n_sims + 1

        names_target = action['targets']
        pos_act  = action[name_actor]['pos']
        yaw_act  = action[name_actor]['yaw']
        velocity = action[name_actor]['velocity']
        duration = action[name_actor]['duration']

        # place objects
        for name_target in names_target:
            pos_obj  = action[name_target]['pos']
            yaw_obj  = action[name_target]['yaw']
            sim.setObjectPose(name_target, pos_obj[0], pos_obj[1], yaw_obj)

        vec = np.array(velocity)/np.linalg.norm(np.array(velocity))
        sim.setObjectPose( name_actor, 
                           pos_act[0]-vec[0]*dims_actor[0], \
                           pos_act[1]-vec[1]*dims_actor[1], yaw_act)

        if is_vis: # fill Colors
            for name_target in names_target:
                cellinfo = sim.getCellBodyProperties(name_target)
                for info in cellinfo:
                    info['color'] = matplotlib.cm.coolwarm(cnormM(info['mass']))
                    #info['color'] = [matplotlib.cm.gray(cnormM(info['mass']))[0],0,\
                    #                 matplotlib.cm.gray(cnormf(info['fric']))[0],1]
                    #info['color'] = matplotlib.cm.coolwarm(cnormf(info['fric']))

                sim.setCellBodyProperties(name_target,cellinfo)

        if len(velocity)==2:
            velocity = [velocity[0],velocity[1],0]

        ret = sim.applyConstantVelocity(name_actor,velocity,duration,
                                        traj_names=traj_names,traj_ref=traj_ref)

        if is_dbg and 'pos_res' in action[names_target[0]]:
            xs={}
            ys={} 
            yaws={}
            for name_target in names_target:
                xs[name_target],ys[name_target],yaws[name_target]\
                 = sim.getObjectPose(name_target)

            for i in range(100):
                for name_target in names_target:
                    x_res = float(action[name_target]['pos_res'][0])
                    y_res = float(action[name_target]['pos_res'][1])
                    yaw_res = float(action[name_target]['yaw_res'])
                    sim.setObjectPose( name_target, x_res,y_res,yaw_res )
                sim.stepSimulation()

                for name_target in names_target:
                    sim.setObjectPose( name_target, xs[name_target],ys[name_target],yaws[name_target] )
                sim.stepSimulation()

        #self.time_sims = self.time_sims + (time.time() - time_start)
        return ret

    def pixel2meter(self, action):
        cells = self.meta_targets[action['targets'][0]]['cells']
        x_px = action[self.name_actor]['pos_px'][0]
        y_px = action[self.name_actor]['pos_px'][1]
        x = -y_px * cells['meter_per_pixel'] + cells['offset'][0]
        y = -x_px * cells['meter_per_pixel'] + cells['offset'][1]
        action[self.name_actor]['pos'] = [x,y]

    def setup(self, meta_setup, n_group=-1):

        # add actor
        if self.is_rec:
            self.sim.recordStart(self.fp_video)

        self.meta_targets = {}
        self.cellinfo_targets = {}
        for meta in meta_setup['target_infos']:
            name = meta['name']
            img_label = meta['label']
            cells = self.createCells(img_label, meta['masses'], meta['meter_per_pixel'], n_group=n_group)
            self.sim.addObject(name,cells, [0,0,0,1])
            self.meta_targets[name] = {'cells':cells}
            self.cellinfo_targets[name] = self.sim.getCellBodyProperties(name)

        if self.is_vis:
            cellinfos = self.sim.getCellBodyProperties(name)
            for c, info in enumerate(cellinfos):
                if c%3==0:
                    info['color'] = [0.4,0.4,0.4,1]
                elif c%3==1:
                    info['color'] = [0.6,0.6,0.6,1]
                else:
                    info['color'] = [0.5,0.5,0.5,1]
            self.sim.setCellBodyProperties(name, cellinfos)
            
            if self.is_rec:
                time.sleep(2)

        self.name_actor = meta_setup['actor']['name']
        self.sim.addObject( self.name_actor, meta_setup['actor']['type'],
                            [1,1,0,1] )
        self.dims_actor = self.sim.shapes[meta_setup['actor']['type']]['dims']
        self.mass_minmax = meta_setup['mass_minmax']
        self.fric_minmax = [0.1,1]

        # Train
        if 'real' in meta_setup:
            self.actions_train = []
            self.actions_test = []

            for idx in meta_setup['train']:
                self.actions_train.append(meta_setup['real'][idx])
            for idx in meta_setup['test']:
                self.actions_test.append(meta_setup['real'][idx])

            self.poses_refs = []
            for action in self.actions_train:
                self.poses_refs.append({})
                for obj in action['targets']:
                    x_res = float(action[obj]['pos_res'][0])
                    y_res = float(action[obj]['pos_res'][1])
                    yaw_res = float(action[obj]['yaw_res'])
                    self.sim.setObjectPose(obj,x_res,y_res,yaw_res)
                    cellpos = self.sim.getCellBodyPose(obj)
                    self.poses_refs[-1][obj]\
                     = {'x':x_res,'y':y_res,'yaw':yaw_res,'cellpos':cellpos}

            self.poses_gts = []
            for action in self.actions_test:
                self.poses_gts.append({})
                for obj in action['targets']:
                    x_res = float(action[obj]['pos_res'][0])
                    y_res = float(action[obj]['pos_res'][1])
                    yaw_res = float(action[obj]['yaw_res'])
                    self.sim.setObjectPose(obj,x_res,y_res,yaw_res)
                    cellpos = self.sim.getCellBodyPose(obj)
                    self.poses_gts[-1][obj]\
                     = {'x':x_res,'y':y_res,'yaw':yaw_res,'cellpos':cellpos}

        else:
            self.actions_train = meta_setup['train']
            for i in range(len(self.actions_train)):
                if 'pos_px' in self.actions_train[i][self.name_actor]:
                    self.pixel2meter(self.actions_train[i])

            self.poses_refs = []
            for action in self.actions_train:

                if self.is_dbg:
                    self.debugId = self.sim.addUserDebugText('Reference Simulation %d'%(len(self.poses_refs)+1),[0.2,0,0.2],[0,0,0],1,0, self.debugId)

                #self.simulate(action)
                self.simulate(self.sim, action, self.name_actor, self.dims_actor)

                self.poses_refs.append({})
                for name in action['targets']:
                    x,y,yaw = self.sim.getObjectPose(name)
                    cellpos = self.sim.getCellBodyPose(name)
                    self.poses_refs[-1][name]\
                     = {'x':x,'y':y,'yaw':yaw,'cellpos':cellpos}
                    action[name]['pos_res'] = (x,y)
                    action[name]['yaw_res'] = yaw

            # Test
            self.actions_test = meta_setup['test']
            for i in range(len(self.actions_test)):
                if 'pos_px' in self.actions_test[i][self.name_actor]:
                    self.pixel2meter(self.actions_test[i])

            self.poses_gts = []
            for i, action in enumerate(self.actions_test):

                if self.is_dbg:
                    self.debugId = self.sim.addUserDebugText('Testing Simulation %d'%(len(self.poses_gts)+1),[0.2,0,0.2],[0,0,0],1,0, self.debugId)

                gt = {}
                for name in action['targets']:
                    self.sim.setCellBodyProperties(name,self.cellinfo_targets[name])
                #self.simulate(action)
                self.simulate(self.sim, action, self.name_actor, self.dims_actor)

                for name in action['targets']:
                    x,y,yaw = self.sim.getObjectPose(name)
                    cellpos = self.sim.getCellBodyPose(name)
                    gt[name] = {}
                    gt[name]['x'] = x
                    gt[name]['y'] = y
                    gt[name]['yaw'] = yaw
                    gt[name]['cellpos'] = cellpos                    
                self.poses_gts.append(gt)

    def infer(self, n_iters=20, timelimit=np.inf):
        pass

    def evaluate(self, cellinfos):

        poses_all = []
        errs_xy = []
        errs_yaw = []
        errs_cell = []
        for i, action in enumerate(self.actions_test):

            for name in action['targets']:
                self.sim.setCellBodyProperties(name,cellinfos[name])
            #self.simulate(action)
            self.simulate(self.sim, action, self.name_actor, self.dims_actor)

            poses_est = {}
            for name in action['targets']:
                x,y,yaw = self.sim.getObjectPose(name)
                cellpos = self.sim.getCellBodyPose(name)

                poses_est[name] = {}
                poses_est[name]['x'] = x
                poses_est[name]['y'] = y
                poses_est[name]['yaw'] = yaw
                poses_est[name]['cellpos'] = cellpos

                x_gt       = self.poses_gts[i][name]['x']
                y_gt       = self.poses_gts[i][name]['y']
                yaw_gt     = self.poses_gts[i][name]['yaw']
                cellpos_gt = self.poses_gts[i][name]['cellpos']

                errs_xy.append(np.linalg.norm(np.array([x_gt-x,y_gt-y])))
                errs_yaw.append((36000+abs(yaw_gt-yaw)/np.pi*18000)%36000*0.01)
                errs_cell.append(self.distanceCellPos(cellpos_gt, cellpos))

            poses_all.append({})
            poses_all[-1]['gt'] = self.poses_gts[i]
            poses_all[-1]['estimation'] = poses_est

        print('  ', errs_cell)

        summary = { 'error_xy'  :np.mean(np.array(errs_xy)),
                    'error_yaw' :np.mean(np.array(errs_yaw)),
                    'error_cell':np.mean(np.array(errs_cell)),
                    'errors_xy':errs_xy,
                    'errors_yaw':errs_yaw,
                    'errors_cell':errs_cell,
                    'poses':poses_all
                  }
        return summary
