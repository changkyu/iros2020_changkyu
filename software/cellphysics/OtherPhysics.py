import time
import numpy as np
import cma
import scipy
from PhysicsBase import CellPhysicsBase
from DiffPhysics import DiffPhysics

class FiniteDiffPhysics(DiffPhysics):

    def __init__(self, is_dbg=False, fp_video='', diff_unit=0.5):
        DiffPhysics.__init__(self,is_dbg=is_dbg,fp_video=fp_video)
        self.diff_unit = diff_unit

    def computeGrad(self, name, cellpos_ref, cellpos_cur,  
                          cellinfo, action, infer_type):

        cellerr_cur = self.distanceCellPos(cellpos_ref,cellpos_cur)

        grad_masses = []
        if infer_type=='cell_mass':
            for c in range(len(cellinfo)):
                cellinfo_nei = []
                for info in cellinfo:
                    cellinfo_nei.append(info.copy())

                cellinfo_nei[c]['mass'] = cellinfo[c]['mass'] + self.diff_unit

                self.sim.setCellBodyProperties(name,cellinfo_nei)
                self.simulate(action)
                cellpos_nei = self.sim.getCellBodyPose(name)
                cellerr_nei = self.distanceCellPos(cellpos_ref,cellpos_nei)

                if abs(cellerr_nei - cellerr_cur) > 0.001:
                    grad = cellerr_nei - cellerr_cur
                else:
                    grad = 0
                grad_masses.append(grad)
        else:
            grad_masses = [0] * len(cellinfo)
            group2cell = self.meta_targets[name]['cells']['group2cell']            
            for g in group2cell:
                cellinfo_nei = []
                for info in cellinfo:
                    cellinfo_nei.append(info.copy())

                for c in group2cell[g]:
                    cellinfo_nei[c]['mass'] = cellinfo[c]['mass'] + self.diff_unit

                self.sim.setCellBodyProperties(name,cellinfo_nei)                
                self.simulate(action)
                cellpos_nei = self.sim.getCellBodyPose(name)
                cellerr_nei = self.distanceCellPos(cellpos_ref,cellpos_nei)
                
                if abs(cellerr_nei - cellerr_cur) > 0.001:
                    grad = cellerr_nei - cellerr_cur
                else:
                    grad = 0
                grad = cellerr_nei - cellerr_cur
                for c in group2cell[g]:
                    grad_masses[c] = grad

        #print(grad_masses)

        self.sim.setCellBodyProperties(name,cellinfo)

        return grad_masses

    def updateProperties(self, name, cellinfo, cellpos_refs, cellpos_nows, 
                               actions, infer_type, lr=0.1, method='mean'):
        
        n_cells = len(cellinfo)
        n_expr = len(cellpos_refs)

        if infer_type=='cell_mass':
            i = 0
            grad_all = []
            for cellpos_ref, cellpos_now in zip(cellpos_refs, cellpos_nows):
                grad = self.computeGrad(name, cellpos_ref, cellpos_now, \
                                        cellinfo, actions[i], infer_type)
                
                grad_all.append(grad)
                i = i + 1

            if method=='max':
                idx_max = np.argmax(np.abs(np.array(grad_all)),axis=0)
                grad = np.diag(np.array(grad_all)[idx_max])
            elif method=='mean':
                grad = np.mean(np.array(grad_all),axis=0)

            for c in range(len(cellinfo)):
                cellinfo[c]['mass'] = cellinfo[c]['mass'] - grad[c] * lr
        else:
            i = 0
            grad_all = []
            grad_group = {}            
            cell2group = self.meta_targets[name]['cells']['cell2group']
            for cellpos_ref, cellpos_now in zip(cellpos_refs, cellpos_nows):
                grad = self.computeGrad(name, cellpos_ref, cellpos_now,\
                                        cellinfo, actions[i], infer_type)
                grad_all.append(grad)
                i = i + 1

                for c, gr in enumerate(grad):
                    g = cell2group[c]
                    if g not in grad_group:
                        grad_group[g] = []
                    grad_group[g].append(gr)
                
            for g in grad_group:
                grad_group[g] = np.mean(np.array(grad_group[g]))

            for c in range(len(cellinfo)):
                cellinfo[c]['mass'] = cellinfo[c]['mass']\
                                       - grad_group[cell2group[c]] * lr

        return cellinfo

    def infer(self, n_iters=100, infer_type='cell_mass', timelimit=np.inf,
                    nsimslimit=np.inf ):

        # initialize
        masses = []
        for name in self.meta_targets:
            cellinfo = self.sim.getCellBodyProperties(name)
            for info in cellinfo:
                masses.append((self.mass_minmax[1]-self.mass_minmax[0])*0.5)
                info['mass'] = masses[-1]
            self.sim.setCellBodyProperties(name,cellinfo)

        name2cellpos_refs = {}
        for i, poses_ref in enumerate(self.poses_refs):
            for name in poses_ref:
                if name not in name2cellpos_refs:                    
                    name2cellpos_refs[name] = []
                name2cellpos_refs[name].append(poses_ref[name]['cellpos'])

        name2cellpos_inits = {}
        for name in self.meta_targets:
            name2cellpos_inits[name] = []

        for i, action in enumerate(self.actions_train):
            for name in action['targets']:
                pos_obj  = action[name]['pos']
                yaw_obj  = action[name]['yaw']
                self.sim.setObjectPose(name, pos_obj[0], pos_obj[1], yaw_obj)
                cellpos = self.sim.getCellBodyPose(name)
                name2cellpos_inits[name].append(cellpos)            

        # infer
        history = []
        self.n_sims = 0
        time_start = time.time()
        err_min = np.inf
        cellinfos_evals = []
        it = 0
        while it <= n_iters:
            errs_xy = []
            errs_yaw = []
            errs_cell = []

            name2cellpos_nows = {}
            for name in self.meta_targets:
                name2cellpos_nows[name] = []
            
            for i, action in enumerate(self.actions_train):

                if self.is_dbg:
                    text_debug = 'iter: %d action: %d\n' % (it,i+1)

                self.simulate(action)
                
                for name in action['targets']:
                    x,y,yaw = self.sim.getObjectPose(name)
                    cellpos = self.sim.getCellBodyPose(name)
                    name2cellpos_nows[name].append(cellpos)

                    x_ref       = self.poses_refs[i][name]['x']
                    y_ref       = self.poses_refs[i][name]['y']
                    yaw_ref     = self.poses_refs[i][name]['yaw']
                    cellpos_ref = self.poses_refs[i][name]['cellpos']

                    errs_xy.append(np.linalg.norm(np.array([x_ref-x,y_ref-y])))
                    errs_yaw.append((36000+abs(yaw_ref-yaw)/np.pi*18000)%36000*0.01)
                    errs_cell.append(self.distanceCellPos(cellpos_ref,cellpos))

                    if self.is_dbg:
                        text_debug = text_debug +\
                                     'error [%s]: %.3f(m) %.3f(deg)\n'%\
                                     (name,errs_xy[-1],errs_yaw[-1])

                if self.is_dbg:
                    self.debugId = self.sim.addUserDebugText(
                                       text_debug,[0.2,0,0.2],[0,0,0],1,0,
                                       replaceItemUniqueId=self.debugId )

            cellinfos = {}
            for name in self.meta_targets:
                cellinfo = self.sim.getCellBodyProperties(name)

                if it > 0:
                    cellinfo = self.updateProperties( name,
                                                      cellinfo, 
                                                      name2cellpos_refs[name],
                                                      name2cellpos_nows[name], 
                                                      self.actions_train,
                                                      infer_type,
                                                      lr=1 * 0.9**it
                                                    )

                for info in cellinfo:
                    if self.mass_minmax[0] > info['mass']:
                        info['mass'] = self.mass_minmax[0]
                    elif self.mass_minmax[1] < info['mass']:
                        info['mass'] = self.mass_minmax[1]

                self.sim.setCellBodyProperties(name,cellinfo)
                cellinfos[name] = cellinfo
        
            err_xy = np.mean(np.array(errs_xy))
            err_yaw = np.mean(np.array(errs_yaw))
            err_cell = np.mean(np.array(errs_cell))            
            if err_min > err_cell:
                err_min = err_cell                
                cellinfos_best = cellinfos
                history.append({'n_sims':self.n_sims,
                                'time':time.time()-time_start,
                                'train_error_xy':err_xy,
                                'train_error_yaw':err_yaw,
                                'train_error_cell':err_cell,
                                'cellinfos':cellinfos,
                               })

            print('iter:%d\t cell err: %.3f pose err: %.3f (m) %.3f (deg)'%\
                  (it,err_cell, err_xy, err_yaw))

            it = it + 1

            if time.time()-time_start > timelimit:
                break

            if self.n_sims > nsimslimit:
                break

        self.cellinfos_infer = cellinfos_best
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



class RandomSearchPhysics(CellPhysicsBase):

    def __init__(self, is_dbg=False, fp_video=''):
        CellPhysicsBase.__init__(self,is_dbg=is_dbg,fp_video=fp_video)

    def infer(self, n_iters=100, infer_type='cell_mass', grid_resolution=10, 
                    timelimit=np.inf, nsimslimit=np.inf):
        
        name2cellpos_refs = {}
        for i, poses_ref in enumerate(self.poses_refs):
            for name in poses_ref:
                if name not in name2cellpos_refs:
                    name2cellpos_refs[name] = []
                name2cellpos_refs[name].append(poses_ref[name]['cellpos'])

        it = 0
        history = []
        self.n_sims = 0
        time_start = time.time()
        err_min = np.inf
        running = True
        while running:
            if it > 0:
                if infer_type=='cell_mass':
                    for name in self.meta_targets:
                        cellinfo = self.sim.getCellBodyProperties(name)
                        for info in cellinfo:
                            info['mass'] = np.random.rand()\
                                           *(self.mass_minmax[1]-self.mass_minmax[0])\
                                           + self.mass_minmax[0]
                        self.sim.setCellBodyProperties(name,cellinfo)
                elif infer_type=='group_mass':
                    for name in self.meta_targets:
                        cellinfo = self.sim.getCellBodyProperties(name)
                        for g in self.meta_targets[name]['cells']['group']:
                            for c in self.meta_targets[name]['cells']['group2cell'][g]:
                                cellinfo[c]['mass'] = np.random.rand()\
                                                      *(self.mass_minmax[1]-self.mass_minmax[0])\
                                                      + self.mass_minmax[0]
                        self.sim.setCellBodyProperties(name,cellinfo)
                else:
                    NotImplementedError("%s is not implemented yet" % infer_type)
            else:
                for name in self.meta_targets:
                    cellinfo = self.sim.getCellBodyProperties(name)
                    for info in cellinfo:
                        info['mass'] = (self.mass_minmax[1]-self.mass_minmax[0])*0.5
                    self.sim.setCellBodyProperties(name,cellinfo)

            name2cellpos_nows = {}
            for name in self.meta_targets:
                name2cellpos_nows[name] = []

            errs_xy = []
            errs_yaw = []
            for i, action in enumerate(self.actions_train):
        
                self.simulate(action)

                for name in action['targets']:
                    x,y,yaw = self.sim.getObjectPose(name)
                    cellpos = self.sim.getCellBodyPose(name)
                    name2cellpos_nows[name].append(cellpos)

                    x_ref   = self.poses_refs[i][name]['x']
                    y_ref   = self.poses_refs[i][name]['y']
                    yaw_ref = self.poses_refs[i][name]['yaw']                

                    errs_xy.append(np.linalg.norm(np.array([x_ref-x,y_ref-y])))
                    errs_yaw.append((36000+abs(yaw_ref-yaw)/np.pi*18000)%36000*0.01)
            
            errs_cell = []
            cellinfos = {}
            for name in self.meta_targets:
                cellinfos[name] = self.sim.getCellBodyProperties(name)
                for cellpos_ref, cellpos_now in zip(name2cellpos_refs[name],\
                                                    name2cellpos_nows[name]):
                    errs_cell.append(self.distanceCellPos(cellpos_ref,cellpos_now))
                
            err_xy = np.mean(np.array(errs_xy))
            err_yaw = np.mean(np.array(errs_yaw))
            err_cell = np.mean(np.array(errs_cell))            
            if err_min > err_cell:
                err_min = err_cell                
                cellinfos_best = cellinfos
                history.append({'n_sims':self.n_sims,
                                'time':time.time()-time_start,
                                'error_xy'  :err_xy,
                                'error_yaw' :err_yaw,
                                'error_cell':err_cell,
                                'cellinfos' :cellinfos
                               })

            print('iter:%d\t cell err: %.3f pose err: %.3f (m) %.3f (deg)'%\
                  (it,err_cell, err_xy, err_yaw))
            it = it + 1

            if timelimit < time.time()-time_start:
                running = False
                break

            if nsimslimit < self.n_sims:
                running = False
                break

        self.cellinfos_infer = cellinfos_best
        time_end = time.time()
        self.time_infer = time_end - time_start

        # evaluation
        for i, h in enumerate(history):
            summary = self.evaluate(h['cellinfos'])
            print('eval:%d\t cell err: %.3f pose err: %.3f (m) %.3f (deg)'%\
                  (i,summary['error_cell'], summary['error_xy'], summary['error_yaw']))
            h['test_error_xy']    = summary['error_xy']
            h['test_error_yaw']   = summary['error_yaw']
            h['test_error_cell']  = summary['error_cell']
            h['test_errors_xy']   = summary['errors_xy']
            h['test_errors_yaw']  = summary['errors_yaw']
            h['test_errors_cell'] = summary['errors_cell']
            h['test_poses']       = summary['poses']
        return history


class GaussianRandomPhysics(CellPhysicsBase):

    def __init__(self, is_dbg=False, fp_video=''):
        CellPhysicsBase.__init__(self,is_dbg=is_dbg,fp_video=fp_video)

    def infer(self, n_iters=100, infer_type='cell_mass', 
                    timelimit=np.inf, nsimslimit=np.inf):
        
        raduis = (self.mass_minmax[1]-self.mass_minmax[0])*0.5
        
        name2cellpos_refs = {}
        for i, poses_ref in enumerate(self.poses_refs):
            for name in poses_ref:
                if name not in name2cellpos_refs:
                    name2cellpos_refs[name] = []
                name2cellpos_refs[name].append(poses_ref[name]['cellpos'])
        
        masses_min = []
        if infer_type=='cell_mass':            
            for name in self.meta_targets:
                cellinfo = self.sim.getCellBodyProperties(name)
                for info in cellinfo:
                    masses_min.append((self.mass_minmax[1]-self.mass_minmax[0])*0.5)
                self.sim.setCellBodyProperties(name,cellinfo)
        elif infer_type=='group_mass':
            for name in self.meta_targets:
                cellinfo = self.sim.getCellBodyProperties(name)
                for g in self.meta_targets[name]['cells']['group']:
                    for c in self.meta_targets[name]['cells']['group2cell'][g]:
                        masses_min.append((self.mass_minmax[1]-self.mass_minmax[0])*0.5)
                self.sim.setCellBodyProperties(name,cellinfo)
        else:
            NotImplementedError("%s is not implemented yet" % infer_type)
        
        masses = list(masses_min)
        it = 0
        history = []
        self.n_sims = 0
        time_start = time.time()
        running = True
        err_min = np.inf
        while running and raduis > 0.1:            
            for j in range(10):
                if it > 0:
                    if infer_type=='cell_mass':
                        m = 0
                        for name in self.meta_targets:
                            cellinfo = self.sim.getCellBodyProperties(name)
                            for info in cellinfo:
                                while True:
                                    masses[m] = np.random.randn()*raduis + masses_min[m]
                                    if masses[m] >= self.mass_minmax[0] and\
                                       masses[m] <= self.mass_minmax[1]:
                                        break
                                info['mass'] = masses[m]
                                m = m + 1                            
                            self.sim.setCellBodyProperties(name,cellinfo)
                    elif infer_type=='group_mass':
                        m = 0
                        for name in self.meta_targets:
                            cellinfo = self.sim.getCellBodyProperties(name)
                            for g in self.meta_targets[name]['cells']['group']:
                                for c in self.meta_targets[name]['cells']['group2cell'][g]:
                                    while True:
                                        masses[m] = np.random.randn()*raduis + masses_min[m]
                                        if masses[m] >= self.mass_minmax[0] and\
                                           masses[m] <= self.mass_minmax[1]:
                                            break
                                    cellinfo[c]['mass'] = masses[m]
                                    m = m + 1
                            self.sim.setCellBodyProperties(name,cellinfo)
                    else:
                        NotImplementedError("%s is not implemented yet" % infer_type)
                else:
                    for name in self.meta_targets:
                        cellinfo = self.sim.getCellBodyProperties(name)
                        for info in cellinfo:
                            info['mass'] = (self.mass_minmax[1]-self.mass_minmax[0])*0.5
                        self.sim.setCellBodyProperties(name,cellinfo)
                
                name2cellpos_nows = {}
                for name in self.meta_targets:
                    name2cellpos_nows[name] = []

                errs_xy = []
                errs_yaw = []
                for i, action in enumerate(self.actions_train):
            
                    self.simulate(action)

                    for name in action['targets']:
                        x,y,yaw = self.sim.getObjectPose(name)
                        cellpos = self.sim.getCellBodyPose(name)
                        name2cellpos_nows[name].append(cellpos)

                        x_ref   = self.poses_refs[i][name]['x']
                        y_ref   = self.poses_refs[i][name]['y']
                        yaw_ref = self.poses_refs[i][name]['yaw']

                        errs_xy.append(np.linalg.norm(np.array([x_ref-x,y_ref-y])))
                        errs_yaw.append((36000+abs(yaw_ref-yaw)/np.pi*18000)%36000*0.01)
                
                errs_cell = []
                cellinfos = {}
                for name in self.meta_targets:
                    cellinfos[name] = self.sim.getCellBodyProperties(name)
                    for cellpos_ref, cellpos_now in zip(name2cellpos_refs[name],\
                                                        name2cellpos_nows[name]):
                        errs_cell.append(self.distanceCellPos(cellpos_ref,cellpos_now))
                    
                err_xy = np.mean(np.array(errs_xy))
                err_yaw = np.mean(np.array(errs_yaw))
                err_cell = np.mean(np.array(errs_cell))
                if err_min > err_cell:
                    err_min = err_cell
                    cellinfos_best = cellinfos
                    masses_min = masses
                    history.append({'n_sims':self.n_sims,
                                    'time':time.time()-time_start,
                                    'error_xy'  :err_xy,
                                    'error_yaw' :err_yaw,
                                    'error_cell':err_cell,
                                    'cellinfos' :cellinfos,
                                   })

                if timelimit < time.time()-time_start:
                    running = False
                    break

                if nsimslimit < self.n_sims:
                    running = False
                    break

                print('iter:%d\t cell err: %.3f pose err: %.3f (m) %.3f (deg)'%\
                      (it,err_cell, err_xy, err_yaw))
                it = it + 1

            raduis = raduis * 0.5
        self.cellinfos_infer = cellinfos_best

        # evaluation
        for i, h in enumerate(history):
            summary = self.evaluate(h['cellinfos'])
            print('eval:%d\t cell err: %.3f pose err: %.3f (m) %.3f (deg)'%\
                  (i,summary['error_cell'], summary['error_xy'], summary['error_yaw']))
            h['test_error_xy']    = summary['error_xy']
            h['test_error_yaw']   = summary['error_yaw']
            h['test_error_cell']  = summary['error_cell']
            h['test_errors_xy']   = summary['errors_xy']
            h['test_errors_yaw']  = summary['errors_yaw']
            h['test_errors_cell'] = summary['errors_cell']
            h['test_poses']       = summary['poses']
        return history

class BlackBoxPhysics(CellPhysicsBase):

    def __init__(self, is_dbg=False, fp_video=''):
        CellPhysicsBase.__init__(self,is_dbg=is_dbg,fp_video=fp_video)

    def initOptimizer(self, x0):
        pass

    def stepOptimizer(self, fn_loss):
        pass

    def infer(self, n_iters=100, infer_type='cell_mass', 
                    timelimit=np.inf, nsimslimit=np.inf):
        
        name2cellpos_refs = {}
        for i, poses_ref in enumerate(self.poses_refs):
            for name in poses_ref:
                if name not in name2cellpos_refs:
                    name2cellpos_refs[name] = []
                name2cellpos_refs[name].append(poses_ref[name]['cellpos'])
        
        masses = []
        if infer_type=='cell_mass':            
            for name in self.meta_targets:
                cellinfo = self.sim.getCellBodyProperties(name)
                for info in cellinfo:
                    masses.append((self.mass_minmax[1]-self.mass_minmax[0])*0.5)
        elif infer_type=='group_mass':
            for name in self.meta_targets:
                cellinfo = self.sim.getCellBodyProperties(name)
                for g in self.meta_targets[name]['cells']['group']:
                    for c in self.meta_targets[name]['cells']['group2cell'][g]:
                        masses.append((self.mass_minmax[1]-self.mass_minmax[0])*0.5)
        else:
            NotImplementedError("%s is not implemented yet" % infer_type)
        
        it = 0
        history = []
        self.n_sims = 0
        time_start = time.time()
        running = True
        err_min = [np.inf]

        def loss(masses):

            errs_xy = []
            errs_yaw = []
            errs_cell = []

            for i, action in enumerate(self.actions_train):

                if infer_type=='cell_mass':
                    m = 0
                    for name in self.meta_targets:
                        cellinfo = self.sim.getCellBodyProperties(name)
                        for info in cellinfo:
                            info['mass'] = masses[m]
                            m = m + 1                            
                        self.sim.setCellBodyProperties(name,cellinfo)
                elif infer_type=='group_mass':
                    m = 0
                    for name in self.meta_targets:
                        cellinfo = self.sim.getCellBodyProperties(name)
                        for g in self.meta_targets[name]['cells']['group']:
                            for c in self.meta_targets[name]['cells']['group2cell'][g]:
                                cellinfo[c]['mass'] = masses[m]
                                m = m + 1
                        self.sim.setCellBodyProperties(name,cellinfo)
                else:
                    NotImplementedError("%s is not implemented yet" % infer_type)
        
                self.simulate(action)

                for name in action['targets']:
                    x,y,yaw = self.sim.getObjectPose(name)
                    cellpos = self.sim.getCellBodyPose(name)

                    x_ref   = self.poses_refs[i][name]['x']
                    y_ref   = self.poses_refs[i][name]['y']
                    yaw_ref = self.poses_refs[i][name]['yaw']

                    errs_xy.append(np.linalg.norm(np.array([x_ref-x,y_ref-y])))
                    errs_yaw.append((36000+abs(yaw_ref-yaw)/np.pi*18000)%36000*0.01)
                    errs_cell.append(self.distanceCellPos(name2cellpos_refs[name][i],cellpos))

            err_xy = np.mean(np.array(errs_xy))
            err_yaw = np.mean(np.array(errs_yaw))
            err_cell = np.mean(np.array(errs_cell))
            if err_min[0] > err_cell:
                err_min[0] = err_cell
                cellinfos = {}
                for name in self.meta_targets:
                    cellinfos[name] = self.sim.getCellBodyProperties(name)
                cellinfos_best = cellinfos
                masses_min = masses
                history.append({'n_sims':self.n_sims,
                                'time':time.time()-time_start,
                                'error_xy'  :err_xy,
                                'error_yaw' :err_yaw,
                                'error_cell':err_cell,
                                'cellinfos' :cellinfos,
                               })


                print('nsim:%d\t cell err: %.3f pose err: %.3f (m) %.3f (deg)'%\
                  (self.n_sims,err_cell, err_xy, err_yaw))

            return err_cell

        self.initOptimizer(masses)
        while running:

            self.stepOptimizer(loss)
            it = it + 1

            if timelimit < time.time()-time_start:
                running = False
                break

            if nsimslimit < self.n_sims:
                running = False
                break

        self.cellinfos_infer = history[-1]['cellinfos']

        # evaluation
        for i, h in enumerate(history):
            summary = self.evaluate(h['cellinfos'])
            print('eval:%d\t cell err: %.3f pose err: %.3f (m) %.3f (deg)'%\
                  (i,summary['error_cell'], summary['error_xy'], summary['error_yaw']))
            h['test_error_xy']    = summary['error_xy']
            h['test_error_yaw']   = summary['error_yaw']
            h['test_error_cell']  = summary['error_cell']
            h['test_errors_xy']   = summary['errors_xy']
            h['test_errors_yaw']  = summary['errors_yaw']
            h['test_errors_cell'] = summary['errors_cell']
            h['test_poses']       = summary['poses']
        return history

class CMAPhysics(BlackBoxPhysics):

    def __init__(self, is_dbg=False, fp_video=''):
        BlackBoxPhysics.__init__(self,is_dbg=is_dbg,fp_video=fp_video)

    def initOptimizer(self, x0):
        self.es = cma.CMAEvolutionStrategy(x0, 1.0)        

    def stepOptimizer(self, fn_loss):
        self.es.optimize(fn_loss,iterations=1)
        return self.es.xcurrent

class NelderMeadPhysics(BlackBoxPhysics):

    def __init__(self, is_dbg=False, fp_video=''):
        BlackBoxPhysics.__init__(self,is_dbg=is_dbg,fp_video=fp_video)

    def initOptimizer(self, x0):
        self.x_current = x0        

    def stepOptimizer(self, fn_loss):
        res = scipy.optimize.minimize(fn_loss, self.x_current, method='Nelder-Mead', options={'maxiter':1,'disp':False})
        self.x_current = res.x
        return self.x_current