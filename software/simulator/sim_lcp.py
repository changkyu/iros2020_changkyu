import numpy as np
import torch

dir_qpth = os.path.abspath('../../qpth/')
if os.path.abspath(dir_qpth) not in sys.path:
    sys.path.append(dir_qpth)
import qpth.solvers.pdipm.batch as pdipm
            
class SimLCP:

    eps = 1e-12
    verbose=0
    notImprovedLim=3
    maxIter=20

    def __init__(self, cells_prop, cell_size, idx_ctr):

        self._M = self.M(cells_prop, cell_size)
        self.idx_ctr = idx_ctr

        n = len(cells_prop)
        self.mu = np.zeros(n*2)
        for c, prop in enumerate(cells_prop):
            self.mu[c] = prop['fric']
            self.mu[c+n] = 1e-5

    def M(self, cells_prop, cell_size):        

        n = len(cells_prop)
        M = np.zeros([3*n,3*n])
        for c, prop in enumerate(cells_prop):
            m = prop['mass']
            I = m * (cell_size**2) * 2 / float(12)
            M_i = np.array([[I,0,0],[0,m,0],[0,0,m]])
            M[3*c:3*(c+1),3*c:3*(c+1)] = M_i

        return M

    def Je(self, cellpos):

        n = len(cellpos)
        x_ctr = cellpos[self.idx_ctr][1]
        y_ctr = cellpos[self.idx_ctr][2]

        Je = np.zeros([(n-1)*2,3*n])
        i = 0
        for c, pos in enumerate(cellpos):
            x = pos[1]
            y = pos[2]
            if c != self.idx_ctr:
                Je_1 = np.array([[-(y_ctr-y), 1, 0],\
                                 [ (x_ctr-x), 0, 1]])
                Je_c = np.array([[         0,-1, 0],\
                                 [         0, 0,-1]])
                Je[2*i:2*(i+1),3*c           :3*(c           +1)] = Je_1
                Je[2*i:2*(i+1),3*self.idx_ctr:3*(self.idx_ctr+1)] = Je_c
                i = i + 1

        return Je

    def Jf(self, cellvel):

        n = len(cellvel)
        Jf = np.zeros([2*n,3*n])
        for c, vel in enumerate(cellvel):
            Jf[c,  3*c+1:3*(c+1)] = -vel[1:2]/np.linalg.norm(vel[1:2])

        for c, vel in enumerate(cellvel):
            Jf[c+n,3*c] = 1 if vel[0] < 0 else -1

        return Jf

    def forward(self, cellpos, cellvel, cellimpulse):

        Je = self.Je(cellpos)

        Q = torch.tensor(self._M).unsqueeze(0)
        p = torch.tensor(cellvel.reshape(-1) + cellimpulse.reshape(-1)).unsqueeze()
        G = torch.tensor(np.stack([Jf, np.zeros(Jf)]))
        h = torch.tensor(np.stack([np.zeros(Jf.shape[0]),self.mu]))
        A = torch.tensor()

        Q_LU, S_LU, R = pdipm_b.pre_factor_kkt(Q, G, A)
        zhats, nus, lams, slacks\
         = pdipm_b.forward( Q, p, G, h, A, b, Q_LU, S_LU, R,
                            eps, verbose, notImprovedLim, maxIter )

    def estimate(self, cellpos_curs, cellpos_gts):

        F_ext = []
        for i in range(1,len(cellpos_curs)):
            cellpos_cur = cellpos_curs[i]
            cellpos_gt  = cellpos_gts[i]
            cellvel_cur = np.array(cellpos_curs[i])-np.array(cellpos_curs[i-1])
            cellvel_gt  = np.array(cellpos_gts[i]) -np.array(cellpos_gts[i-1])

            Je = self.Je(cellpos_cur, self.idx_ctr)            

            cellvel = cellvel_cur.copy()
            while True:
                err = np.linalg.norm(cellvel - cellvel_gt)
                if err < 1e-5:
                    break
