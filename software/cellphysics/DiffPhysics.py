import time
import scipy
import numpy as np
import argparse
from PhysicsBase import CellPhysicsBase

class DiffPhysics(CellPhysicsBase):

    def __init__(self, is_dbg=False, fp_video=''):
        CellPhysicsBase.__init__(self,is_dbg=is_dbg,fp_video=fp_video)

        self.n_grads = 0
        self.time_grads = 0
    
    def M(self, obj, masses):

        cells = self.meta_targets[obj]['cells']['cells']
        cell_size = self.meta_targets[obj]['cells']['cell_size']
        
        n = len(cells)
        M = np.zeros([3*n,3*n])
        for c, m in enumerate(masses):
            m = 1.0
            I = m * (cell_size**2) * 2 / float(12)
            M_i = np.array([[I,0,0],[0,m,0],[0,0,m]])
            M[3*c:3*(c+1),3*c:3*(c+1)] = M_i

        return M

    def Je(self, obj, cellpos):

        c_ctr = self.meta_targets[obj]['cells']['idx_center']
        n = len(cellpos)

        x_ctr = cellpos[c_ctr][1]
        y_ctr = cellpos[c_ctr][2]

        Je = np.zeros([3*(n-1),3*n])
        i = 0
        for c, pos in enumerate(cellpos):
            x = pos[1]
            y = pos[2]
            if c != c_ctr:
                Je_1 = np.array([[-(y_ctr-y), 1, 0],\
                                 [ (x_ctr-x), 0, 1],
                                 [         1, 0, 0]])
                Je_c = np.array([[         0,-1, 0],\
                                 [         0, 0,-1],
                                 [         1, 0, 0]])
                Je[3*i:3*(i+1),3*c    :3*(c    +1)] = Je_1
                Je[3*i:3*(i+1),3*c_ctr:3*(c_ctr+1)] = Je_c
                i = i + 1

        return Je

    def Jf(self, obj, cellvel):

        cell_size = self.meta_targets[obj]['cells']['cell_size']

        n = len(cellvel)
        Jf = np.zeros([2*n,3*n])
        for c, vel in enumerate(cellvel):
            Jf[2*c,  3*c] = 1 if vel[0] < 0 else -1
            Jf[2*c+1,3*c+1:3*(c+1)] = -vel[1:2]/np.linalg.norm(vel[1:2])
        
        return Jf

    def Mu_c(self, obj):

        cells = self.meta_targets[obj]['cells']['cells']
        n = len(cells)
        C = np.zeros([2*n,3*n])
        for c in range(n):
            C[2*c,  3*c] = 1
            C[2*c+1,3*c+1:3*(c+1)] = 0.5
        return C

    def computeGradM(self, name, masses, cellpos_ref, traj_now):

        time_start = time.time()
        def lVlM(M, Je, Jf, Mu_c, Vtdt, Vt):
            M_inv = np.diag(1.0/np.diag(M))
            JeT = Je.transpose()
            X11 = M_inv + M_inv.dot(JeT).dot(np.linalg.inv(-Je.dot(M_inv).dot(JeT))).dot(Je).dot(M_inv)
            return X11.dot(Jf.transpose().dot(Mu_c) - np.diag(Vtdt-Vt))

        M = self.M(name, masses)
        Mu_c = self.Mu_c(name)

        dldV = (np.array(traj_now['cellpos'][-1])-np.array(cellpos_ref)).reshape(-1)

        X = np.zeros(M.shape[0])
        cellvel_prv = np.array(traj_now['cellpos'][-1])-np.array(traj_now['cellpos'][0])
        cellvel_cur = np.array(traj_now['cellpos'][-1])-np.array(traj_now['cellpos'][0])
        Je = self.Je(name, traj_now['cellpos'][-1])
        Jf = self.Jf(name, cellvel_cur)
        X = lVlM(M, Je, Jf, Mu_c, cellvel_cur.reshape(-1), cellvel_prv.reshape(-1))

        grad_masses = dldV.dot(X)
        ret = grad_masses.reshape(-1,3)[:,1].squeeze()

        self.time_grads = self.time_grads + (time.time() - time_start)
        self.n_grads = self.n_grads + 1
        
        return ret

    def computeGradf(self, name, masses, cellpos_ref, traj_now):

        time_start = time.time()
        def lVlf(M, Je, Jf, Mu_c, Vtdt, Vt):
            M_inv = np.diag(1.0/np.diag(M))
            JeT = Je.transpose()
            X11 = M_inv + M_inv.dot(JeT).dot(np.linalg.inv(-Je.dot(M_inv).dot(JeT))).dot(Je).dot(M_inv)

            m = np.diag(np.diag(M).reshape(-1,3)[:,:2].reshape(-1))
            return X11.dot(Jf.transpose()).dot(m)

        M = self.M(name, masses)
        Mu_c = self.Mu_c(name)

        dldV = (np.array(traj_now['cellpos'][-1])-np.array(cellpos_ref)).reshape(-1)

        X = np.zeros(M.shape[0])
        cellvel_prv = np.array(traj_now['cellpos'][-1])-np.array(traj_now['cellpos'][0])
        cellvel_cur = np.array(traj_now['cellpos'][-1])-np.array(traj_now['cellpos'][0])
        Je = self.Je(name, traj_now['cellpos'][-1])
        Jf = self.Jf(name, cellvel_cur)
        X = lVlf(M, Je, Jf, Mu_c, cellvel_cur.reshape(-1), cellvel_prv.reshape(-1))

        ret = dldV.dot(X).reshape(-1,2)[:,1].squeeze()
        
        self.time_grads = self.time_grads + (time.time() - time_start)
        
        return ret

    def infer(self, n_iters=100, infer_type='cell_mass', lr_mass=1.0,
                    cellinfos_init=None, freq_update_fric=np.inf,
                    timelimit=np.inf, nsimslimit=np.inf, errlimit=0, iterlimit=100 ):

        if freq_update_fric==0:
            freq_update_fric = np.inf

        # infer
        history = []
        self.n_sims = 0
        self.n_grads = 0
        self.time_sims = 0
        self.time_grads = 0
        time_start = time.time()
        err_min = [np.inf]
        cache = {}
        def simulate(masses, frics):

            c = 0
            for name in self.meta_targets:
                cellinfo = self.sim.getCellBodyProperties(name)
                for info in cellinfo:
                    info['mass'] = masses[c]
                    info['fric'] = frics[c]
                    c = c + 1
                self.sim.setCellBodyProperties(name,cellinfo)

            errs_xy = []
            errs_yaw = []
            errs_cell = []

            gradM_all = []
            gradf_all = []
            for i, action in enumerate(self.actions_train):

                #traj = self.simulate(action,traj_names=action['targets'])
                self.n_sims = self.n_sims + 1
                time_sims_start = time.time()

                traj = self.simulate( self.sim, action, self.name_actor, self.dims_actor, 
                                      traj_names=action['targets'])
                self.time_sims = self.time_sims + (time.time() - time_sims_start)

                gradM_objs = []
                gradf_objs = []
                for name in action['targets']:
                    x,y,yaw = self.sim.getObjectPose(name)
                    cellpos = self.sim.getCellBodyPose(name)

                    x_ref   = self.poses_refs[i][name]['x']
                    y_ref   = self.poses_refs[i][name]['y']
                    yaw_ref = self.poses_refs[i][name]['yaw']
                    cellpos_ref = self.poses_refs[i][name]['cellpos']

                    errs_xy.append(np.linalg.norm(np.array([x_ref-x,y_ref-y])))
                    errs_yaw.append((36000+abs(yaw_ref-yaw)/np.pi*18000)%36000*0.01)
                    errs_cell.append(self.distanceCellPos(cellpos_ref,cellpos))

                    gradM_obj = self.computeGradM(name,masses,cellpos_ref,traj[name])
                    gradM_objs = gradM_objs + list(gradM_obj)
                    gradf_obj = self.computeGradf(name,masses,cellpos_ref,traj[name])
                    gradf_objs = gradf_objs + list(gradf_obj)

                gradM_all.append(gradM_objs)
                gradf_all.append(gradf_objs)

            gradM = np.mean(np.array(gradM_all),axis=0)
            gradf = np.mean(np.array(gradf_all),axis=0)
            
            err_xy = np.mean(np.array(errs_xy))
            err_yaw = np.mean(np.array(errs_yaw))
            err_cell = np.mean(np.array(errs_cell))

            key = (tuple(masses),tuple(frics))
            cache[key] = {'loss':err_cell,'gradM':gradM,'gradf':gradf}

            if err_min[0] > err_cell:
                err_min[0] = err_cell
                cellinfos = {}
                for name in self.meta_targets:
                    cellinfos[name] = self.sim.getCellBodyProperties(name)
                cellinfos_best = cellinfos
                masses_min = masses
                history.append({'n_sims':self.n_sims,
                                'time':time.time()-time_start,
                                'time_sims':self.time_sims,
                                'n_grads':self.n_grads,
                                'time_grads':self.time_grads,
                                'error_xy'  :err_xy,
                                'error_yaw' :err_yaw,
                                'error_cell':err_cell,
                                'cellinfos' :cellinfos,
                               })

            print('nsim:%d\t cell err: %.3f pose err: %.3f (m) %.3f (deg)'%\
              (self.n_sims,err_cell, err_xy, err_yaw))

        def loss(masses, frics):
            key = (tuple(masses),tuple(frics))
            if key not in cache:
                simulate(masses, frics)
            return cache[key]['loss']

        def gradM(masses, frics):
            key = (tuple(masses),tuple(frics))
            if key not in cache:
                simulate(masses, frics)
            return cache[key]['gradM']

        def gradf(masses, frics):
            key = (tuple(masses),tuple(frics))
            if key not in cache:
                simulate(masses, frics)
            return cache[key]['gradf']

        # initialize
        masses = []
        frics = []
        if cellinfos_init is None:
            for name in self.meta_targets:
                cellinfo = self.sim.getCellBodyProperties(name)
                for c, info in enumerate(cellinfo):
                    masses.append((self.mass_minmax[1]-self.mass_minmax[0])*0.5)
                    frics.append(0.5)

            if self.is_vis:
                for name in self.meta_targets:
                    cellinfo = self.sim.getCellBodyProperties(name)
                    for c, info in enumerate(cellinfo):
                        if c%3==0:
                            info['color'] = [0.4,0.4,0.4,1]
                        elif c%3==1:
                            info['color'] = [0.6,0.6,0.6,1]
                        else:
                            info['color'] = [0.5,0.5,0.5,1]
                    self.sim.setCellBodyProperties(name, cellinfo)

        elif cellinfos_init=='random':
            for name in self.meta_targets:
                cellinfo = self.sim.getCellBodyProperties(name)
                for c, info in enumerate(cellinfo):
                    masses.append((np.random.rand()*0.25+0.375)\
                                  *(self.mass_minmax[1]-self.mass_minmax[0])\
                                  + self.mass_minmax[0])
                    #frics.append(np.random.rand()\
                    #              *(self.fric_minmax[1]-self.fric_minmax[0])\
                    #              + self.fric_minmax[0])
                    frics.append(0.5)

        else:
            for name in self.meta_targets:
                for info in cellinfos_init[name]:
                    masses.append(info['mass'])
                    frics.append(info['fric'])

        it = 1
        masses_cur = list(masses)
        frics_cur = list(frics)
        while it <= n_iters:

            gM = np.array(gradM(masses_cur,frics_cur))
            masses_cur = np.array(masses_cur) - gM*lr_mass*0.9**it
            for c, mass in enumerate(masses_cur):
                if self.mass_minmax[0] > mass:
                    masses_cur[c] = self.mass_minmax[0]
                elif self.mass_minmax[1] < mass:
                    masses_cur[c] = self.mass_minmax[1]
       
            if (it%freq_update_fric)==0 and (self.fric_minmax[1]-self.fric_minmax[0])>0:
                print('update fric')
                gf = np.array(gradf(masses_cur,frics_cur))
                frics_cur = np.array(frics_cur) - gf*(self.fric_minmax[1]-self.fric_minmax[0])*0.9**it
                #frics_cur = np.array(frics_cur) - gf*(self.fric_minmax[1]-self.fric_minmax[0])*0.02
                for c, fric in enumerate(frics_cur):
                    if self.fric_minmax[0] > fric:
                        frics_cur[c] = self.fric_minmax[0]
                    elif self.fric_minmax[1] < fric:
                        frics_cur[c] = self.fric_minmax[1]

            it = it + 1

            if time.time()-time_start > timelimit:
                break

            if self.n_sims > nsimslimit:
                break

            if err_min[0] <= 0.001:
                break

            if it > iterlimit:
                break

        simulate(masses_cur, frics_cur)

        self.cellinfos_infer = history[-1]['cellinfos']
        time_end = time.time()
        self.time_infer = time_end - time_start

        # evaluation
        for i, h in enumerate(history):
            summary = self.evaluate(h['cellinfos'])
            print('eval:%d\t cell err: %.3f pose err: %.3f (m) %.3f (deg)'%\
                  (i,summary['error_cell'], summary['error_xy'], summary['error_yaw']))
            for key in summary:
                h['test_' + key] = summary[key]
        return history

if __name__=='__main__':

    gts = \
    {    
        'book':      {'masses':[1,4],   'goal':[0.63,-0.50,179/180.0*np.pi]},
        'box':       {'masses':[3,1],   'goal':[0.63,-0.50,000/180.0*np.pi]},
        'crimp':     {'masses':[3,1],   'goal':[0.63,-0.50,000/180.0*np.pi]},
        'hammer':    {'masses':[3,1],   'goal':[0.63,-0.54,000/180.0*np.pi]},
        'ranch':     {'masses':[3,1,2], 'goal':[0.63,-0.51,000/180.0*np.pi]},
        'snack':     {'masses':[1,3],   'goal':[0.63,-0.50,179/180.0*np.pi]},
        'toothpaste':{'masses':[3,1],   'goal':[0.63,-0.52,000/180.0*np.pi]},
        'windex':    {'masses':[1,3],   'goal':[0.63,-0.53,179/180.0*np.pi]},
    }
    
    obj2sets = \
    {
        'sims':{'book'       :[9,8,5,2,4],
                'box'        :[3,9,1,7,8],
                'crimp'      :[8,1,9,4,7],
                'hammer'     :[2,8,9,7,3],
                'ranch'      :[4,7,1,2,5],
                'snack'      :[8,9,6,4,2],
                'toothpaste' :[9,4,6,8,7],
                'windex'     :[6,2,4,8,9]},
        'real':{'book'       :[4,8,6,9,0],
                'box'        :[1,5,0,9,2],
                'hammer'     :[9,8,1,6,5],
                'snack'      :[9,1,8,6,5],
                'windex'     :[1,7,6,5,3]}
    }

    idxes_set = [
    [[[0], [1, 2, 3, 4, 5, 6, 7, 8, 9]],
     [[1], [0, 2, 3, 4, 5, 6, 7, 8, 9]],
     [[2], [0, 1, 3, 4, 5, 6, 7, 8, 9]],
     [[3], [0, 1, 2, 4, 5, 6, 7, 8, 9]],
     [[4], [0, 1, 2, 3, 5, 6, 7, 8, 9]],
     [[5], [0, 1, 2, 3, 4, 6, 7, 8, 9]],
     [[6], [0, 1, 2, 3, 4, 5, 7, 8, 9]],
     [[7], [0, 1, 2, 3, 4, 5, 6, 8, 9]],
     [[8], [0, 1, 2, 3, 4, 5, 6, 7, 9]],
     [[9], [0, 1, 2, 3, 4, 5, 6, 7, 8]]],
    [[[2, 6], [0, 1, 3, 4, 5, 7, 8, 9]],
     [[6, 5], [0, 1, 2, 3, 4, 7, 8, 9]],
     [[9, 6], [0, 1, 2, 3, 4, 5, 7, 8]],
     [[8, 9], [0, 1, 2, 3, 4, 5, 6, 7]],
     [[3, 4], [0, 1, 2, 5, 6, 7, 8, 9]],
     [[1, 0], [2, 3, 4, 5, 6, 7, 8, 9]],
     [[0, 7], [1, 2, 3, 4, 5, 6, 8, 9]],
     [[4, 9], [0, 1, 2, 3, 5, 6, 7, 8]],
     [[5, 6], [0, 1, 2, 3, 4, 7, 8, 9]],
     [[0, 3], [1, 2, 4, 5, 6, 7, 8, 9]]],
    [[[6, 4, 9], [0, 1, 2, 3, 5, 7, 8]],
     [[5, 9, 3], [0, 1, 2, 4, 6, 7, 8]],
     [[6, 8, 5], [0, 1, 2, 3, 4, 7, 9]],
     [[4, 9, 8], [0, 1, 2, 3, 5, 6, 7]],
     [[3, 9, 6], [0, 1, 2, 4, 5, 7, 8]],
     [[3, 4, 6], [0, 1, 2, 5, 7, 8, 9]],
     [[7, 8, 1], [0, 2, 3, 4, 5, 6, 9]],
     [[7, 2, 3], [0, 1, 4, 5, 6, 8, 9]],
     [[4, 3, 0], [1, 2, 5, 6, 7, 8, 9]],
     [[0, 5, 8], [1, 2, 3, 4, 6, 7, 9]]],
    [[[0, 6, 7, 2], [1, 3, 4, 5, 8, 9]],
     [[0, 4, 8, 6], [1, 2, 3, 5, 7, 9]],
     [[6, 7, 5, 8], [0, 1, 2, 3, 4, 9]],
     [[7, 4, 2, 8], [0, 1, 3, 5, 6, 9]],
     [[6, 8, 4, 5], [0, 1, 2, 3, 7, 9]],
     [[7, 5, 9, 2], [0, 1, 3, 4, 6, 8]],
     [[4, 1, 7, 8], [0, 2, 3, 5, 6, 9]],
     [[8, 1, 2, 4], [0, 3, 5, 6, 7, 9]],
     [[2, 4, 1, 7], [0, 3, 5, 6, 8, 9]],
     [[7, 0, 4, 3], [1, 2, 5, 6, 8, 9]]],
    [[[1, 6, 8, 5, 9], [0, 2, 3, 4, 7]],
     [[9, 6, 1, 4, 2], [0, 3, 5, 7, 8]],
     [[6, 9, 8, 0, 7], [1, 2, 3, 4, 5]],
     [[6, 4, 8, 5, 0], [1, 2, 3, 7, 9]],
     [[1, 2, 3, 8, 6], [0, 4, 5, 7, 9]],
     [[8, 2, 1, 4, 5], [0, 3, 6, 7, 9]],
     [[8, 5, 9, 7, 3], [0, 1, 2, 4, 6]],
     [[1, 4, 2, 6, 0], [3, 5, 7, 8, 9]],
     [[2, 7, 6, 4, 3], [0, 1, 5, 8, 9]],
     [[2, 6, 9, 5, 7], [0, 1, 3, 4, 8]]],
    ]

    obj2paramntrain =\
    {
        'book':      (7,5),
        'box':       (1,1),
        'crimp':     (9,5),
        'hammer':    (9,4),
        'ranch':     (7,3),
        'snack':     (9,2),
        'windex':    (0,2),
        'toothpaste':(2,4)
    }

    import pickle
    import os

    parser = argparse.ArgumentParser(description='Test')
    parser.add_argument('-o','--outdir',type=str,  default='./sav',help='output directory')
    parser.add_argument('--objects',    type=str,  nargs='+',     help='objects')
    parser.add_argument('--seed',       type=int,  default=0,     help='random seed')
    parser.add_argument('--debug',      type=bool, default=False, help='visualization')
    args = parser.parse_args()

    if args.objects is None or len(args.objects)==0:
        args.objects = objects

    for obj in args.objects:
    
        #obj = 'hammer'
        #idxes_train = [4,1,7,8]
        #idxes_test =  [0,2,3,5,6,9]
        #idxes_train = [1]
        #idxes_test = [1]
        #idxes_train = range(10)
        idxes_test = []
        #idx_param = 9
        
        idx_param = obj2paramntrain[obj][0]
        idxes_train = idxes_set[obj2paramntrain[obj][1]-1][obj2sets['sims'][obj][0]][0]
    
        from setups_sim import get_setups_tools_sim
        setups = get_setups_tools_sim([obj],idxes_train,idxes_test,idx_param)
        setups['target_infos'][0]['masses'] = gts[obj]['masses']
    
        engine = DiffPhysics(is_dbg=False, fp_video='')
        engine.setup(setups, n_group=-1)
        engine.mass_minmax = [2.5 - (10-args.seed)*0.25, 2.5 + (10-args.seed)*0.25]
        engine.fric_minmax = [0.5 - (args.seed-1 )*0.05, 0.5 + (args.seed-1 )*0.05]
        print(engine.fric_minmax)
        history = engine.infer(infer_type='cell_mass',nsimslimit=500, cellinfos_init=None, freq_update_fric=1)
        print('[Test Result] cell err: %.3f (m) pose err: %.3f (m) %.3f (deg)' % 
               (history[-1]['test_error_cell'],
                history[-1]['test_error_xy'],
                history[-1]['test_error_yaw']) )
    
        fp_save = '%s_seed%d.pkl' % (obj,args.seed)
        with open(os.path.join(args.outdir,fp_save),'wb') as f:
            result = {'cellinfo':history[-1]['cellinfos'][obj],
                      'error_cell':history[-1]['error_cell']}
            pickle.dump(result, f, protocol=2)
        
