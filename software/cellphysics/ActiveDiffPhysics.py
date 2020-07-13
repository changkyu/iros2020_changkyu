import os, sys
import numpy as np
import time

from DiffPhysics import DiffPhysics
import multiprocessing
from multiprocessing import Pool

dir_root = os.path.abspath('..')
if os.path.abspath(dir_root) not in sys.path:
    sys.path.append(dir_root)
from simulator import SimBullet

from termcolor import colored, cprint

def infer_multiproc(args):
    meta_setup, cellinfos_init, actions_train, poses_refs, fric_minmax = args

    iterlimit = 10

    meta_setup['train'] = []
    meta_setup['test'] = []    
    estimator = DiffPhysics(is_dbg=False, fp_video='')
    estimator.setup(meta_setup, n_group=-1)
    estimator.actions_train = actions_train
    estimator.poses_refs = poses_refs
    estimator.fric_minmax = fric_minmax

    if fric_minmax is None:
        estimator.fric_minmax = [0.5,0.5]
        iterlimit=np.inf
        nsimslimit=500  
        lr_mass = 0.5     
    elif fric_minmax[1]-fric_minmax[0]==0:
        nsimslimit=np.inf
        iterlimit = iterlimit * 2
        lr_mass = 0.5
    else:
        nsimslimit=np.inf
        lr_mass = 0.5

    history = estimator.infer( infer_type='cell_mass',
                               iterlimit=iterlimit, nsimslimit=nsimslimit,
                               lr_mass=lr_mass,
                               cellinfos_init=cellinfos_init, freq_update_fric=1)
    return history

def poses_refs_multiproc(args):
    meta_setup, action = args

    meta_setup['train'] = [action]
    meta_setup['test'] = []
    estimator = DiffPhysics(is_dbg=False, fp_video='')
    estimator.setup(meta_setup, n_group=-1)
    return estimator.poses_refs[0]

def simulations_multiproc(args):
    name, cells, cellinfo, actions = args

    dims_actor = [0.03,0.03,0.10]
    sim = SimBullet()
    sim.addObject('plane','plane')
    sim.addObject('finger','finger')
    sim.addObject(name,cells)
    sim.setCellBodyProperties(name,cellinfo)

    res = []
    for action in actions:
        DiffPhysics.simulate(sim, action, 'finger', dims_actor)
        x,y,yaw = sim.getObjectPose(name)
        cellpos = sim.getCellBodyPose(name)
        res.append({'x':x,'y':y,'yaw':yaw,'cellpos':cellpos})
    return res

class ActiveDiffPhysics:

    def __init__(self, mode='multi'):
        
        self.mode = mode
        self.pool = Pool(int(multiprocessing.cpu_count()*0.5+0.5))
        
        self.cellinfos_inits = []
        self.frics_minmax = []
        for seed in range(5):
            self.frics_minmax.append([0.5 - seed*0.05, 0.5 + seed*0.05])
            self.cellinfos_inits.append(None)

    def setup(self, meta_setup):
        
        self.name = meta_setup['target_infos'][0]['name']
        self.meta_setup = meta_setup
        if type(meta_setup['train'][0])==int:
            self.actions_train = []
            for idx in meta_setup['train']:
                self.actions_train.append(meta_setup['real'][idx])
        else:
            self.actions_train = list(meta_setup['train'])

        self.estimator = DiffPhysics(is_dbg=False, fp_video='')
        self.estimator.setup(meta_setup, n_group=-1)
        self.meta_targets = self.estimator.meta_targets
        #self.poses_refs = self.pool.map(poses_refs_multiproc,
        #                                [(self.meta_setup, action) 
        #                                 for action in self.actions_train])
        self.poses_refs = self.estimator.poses_refs

    def infer(self, n_train, idx_init):

        actions_in_waiting = list(self.actions_train)
        poses_refs_in_waiting = list(self.poses_refs)
        actions_in_training = []
        poses_refs_in_training = []

        if False:
            idx_sel = -1
            dist_min = np.inf
            for i, action in enumerate(actions_in_waiting):
                dist = np.linalg.norm(action['finger']['pos'])
                if dist_min > dist:
                    dist_min = dist
                    idx_sel = i
        else:
            idx_sel = idx_init

        cellinfos_cur = None
        n_sims = 0
        time_infer = 0
        history = []
        time_start = time.time()
        while len(actions_in_training)<n_train:
            actions_in_training.append(actions_in_waiting[idx_sel])
            poses_refs_in_training.append(poses_refs_in_waiting[idx_sel])
            del actions_in_waiting[idx_sel]
            del poses_refs_in_waiting[idx_sel]

            res_infer = infer_multiproc((self.meta_setup, 
                                         cellinfos_cur, 
                                         actions_in_training, 
                                         poses_refs_in_training, None))
            #cellinfos_cur = res_infer[-1]['cellinfos']
            """
            n_sims = n_sims + res_infer[-1]['n_sims']
            time_infer = time.time() - time_start
            history.append({'n_train':len(actions_in_training),
                            'n_sims':n_sims,
                            'time':time_infer,
                            'error_xy'  :res_infer[-1]['error_xy'],
                            'error_yaw' :res_infer[-1]['error_yaw'],
                            'error_cell':res_infer[-1]['error_cell'],
                            'cellinfos' :res_infer[-1]['cellinfos'],
                            })
            """

            cprint(('error_cell: ', res_infer[-1]['error_cell']), 'green')
            if len(actions_in_waiting)==0:
                break

            if self.mode=='single':
                idx_sel = self.selectAction_single(actions_in_training,
                                                   actions_in_waiting,
                                                   res_infer[-1]['cellinfos'])
            else:
                idx_sel = self.selectAction_multi(actions_in_training, 
                                                  poses_refs_in_training,
                                                  actions_in_waiting,)

        history = res_infer

        # evaluation
        for i, h in enumerate(history):
            summary = self.estimator.evaluate(h['cellinfos'])
            print('eval:%d\t cell err: %.3f pose err: %.3f (m) %.3f (deg)'%\
                  (i,summary['error_cell'], summary['error_xy'], summary['error_yaw']))
            for key in summary:
                h['test_' + key] = summary[key]

        return history

    def selectAction_single(self, actions_in_training, actions_in_waiting, cellinfo):

        res_training = self.pool.map(simulations_multiproc,
                                    [(self.name, 
                                      self.meta_targets[self.name]['cells'], 
                                      cellinfo, 
                                      [action]) 
                                     for action in actions_in_training])

        res_waiting = self.pool.map(simulations_multiproc,
                                    [(self.name, 
                                      self.meta_targets[self.name]['cells'], 
                                      cellinfo, 
                                      [action]) 
                                     for action in actions_in_waiting])

        idx_sel = -1
        dist_sel = -1
        for w in range(len(res_waiting)):
            dist_min = np.inf
            for t in range(len(res_training)):
                dist = DiffPhysics.distanceCellPos(res_waiting[w][0]['cellpos'],
                                                   res_training[t][0]['cellpos'])
                if dist_min > dist:
                    dist_min = dist
            if dist_sel < dist_min:
                dist_sel = dist_min
                idx_sel = w
        return idx_sel

    def selectAction_multi(self, actions_in_training, 
                                 poses_refs_in_training,
                                 actions_in_waiting  ):

        results = self.pool.map(infer_multiproc,
                                [(self.meta_setup,
                                  cellinfos_init,
                                  actions_in_training, 
                                  poses_refs_in_training, 
                                  fric_minmax)
                                  for cellinfos_init, fric_minmax\
                                  in zip(self.cellinfos_inits, self.frics_minmax)])

        #for r, result in enumerate(results):
        #    self.cellinfos_inits[r] = result[-1]['cellinfos']

        cellinfos = []
        for r, result in enumerate(results):
            cellinfos.append(result[-1]['cellinfos'])

        res_sel = self.pool.map(simulations_multiproc,
                                [(self.name, 
                                  self.meta_targets[self.name]['cells'], 
                                  cellinfo, 
                                  actions_in_waiting) 
                                 for cellinfo in cellinfos])
        
        cellpos_std_all = []
        for a in range(len(actions_in_waiting)):
            cellpos_std = np.std([res_sel[i][a]['cellpos'] for i in range(len(res_sel))], axis=0)
            print('cellpos_std.shape:', cellpos_std.shape)                
            cellpos_std_all.append((a,cellpos_std[:,:2].sum()))
        
        idx_sel,_ = max(cellpos_std_all, key=lambda x: x[1])
        cprint(('idx_sel: ', idx_sel, cellpos_std_all), 'red')
        return idx_sel

if __name__=='__main__':


    import pickle
    import argparse

    parser = argparse.ArgumentParser(description='Test')
    parser.add_argument('-o','--outdir',type=str,  default='./sav',help='output directory')
    parser.add_argument('--objects',    type=str,  nargs='+',     help='objects')
    parser.add_argument('--seed',       type=int,  default=0,     help='random seed')
    parser.add_argument('--debug',      type=bool, default=False, help='visualization')
    args = parser.parse_args()

    obj = 'hammer'
    #idxes_train = [4,1,7,8]
    #idxes_test =  [0,2,3,5,6,9]
    #idxes_train = [1,6]
    #idxes_test = [1]
    idxes_train = [4,1,7,8]
    idxes_test = [0,2,3,5,6,9]
    idx_param = 9

    from setups_sim import get_setups_tools_sim
    setups = get_setups_tools_sim([obj],idxes_train,idxes_test,idx_param)

    engine = ActiveDiffPhysics(mode='single')
    engine.setup(setups)
    engine.infer(obj)
    
