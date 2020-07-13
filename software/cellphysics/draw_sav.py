import sys, os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import yaml
import glob
import pickle
import cv2

dir_pusher = os.path.abspath('..')
if os.path.abspath(dir_pusher) not in sys.path:
    sys.path.append(dir_pusher)
from simulator import SimBullet

from PhysicsBase import CellPhysicsBase

if __name__=='__main__':
    #obj = 'hammer'

    dir_sav = 'sav_tmp4'

    for obj in ['book','box','crimp','hammer','ranch','snack','toothpaste','windex']:

        if not os.path.isdir(dir_sav+'/fig'):
            os.makedirs(dir_sav+'/fig')

        fp_label = '../../dataset/objects/%s_label.pgm' % obj
        img_label = cv2.imread(fp_label, cv2.IMREAD_UNCHANGED)
        cells = CellPhysicsBase.createCells(img_label, [1,1,1,1], 0.53/640.0)

        sim = SimBullet(gui=False)
        sim.setupRendering(0.75,-90,-60,500,500)
        sim.addObject(obj,cells, [0,0,1,1])
        sim.setObjectPose(obj, 0, 0, 0)

        files = glob.glob(dir_sav+'/'+obj+'*.pkl')
        for file in files:

            filename = os.path.split(file)[1].split('.')[0]

            f = open(file,'rb')
            sav = pickle.load(f)
            f.close()

            cellinfo = sav['cellinfo']
            error_cell = sav['error_cell']

            masses = []
            frics = []
            forces = []
            for info in cellinfo:
                masses.append(info['mass'])
                frics.append(info['fric'])
                forces.append(info['mass']*info['fric'])

            cnormF = matplotlib.colors.Normalize(vmin=min(forces),vmax=max(forces))
            #cnormF = matplotlib.colors.Normalize(vmin=0.1,vmax=5)
            for info in cellinfo:
                info['color'] = matplotlib.cm.coolwarm(cnormF(info['mass']*info['fric']))
            sim.setCellBodyProperties(obj,cellinfo)
            img = sim.draw()
            img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
            cv2.imwrite(os.path.join(dir_sav+'/fig/' + filename + '_force.png'),img)

            #cnormM = matplotlib.colors.Normalize(vmin=min(masses),vmax=max(masses))
            cnormM = matplotlib.colors.Normalize(vmin=1,vmax=4)
            for info in cellinfo:
                info['color'] = matplotlib.cm.coolwarm(cnormM(info['mass']))
            sim.setCellBodyProperties(obj,cellinfo)
            img = sim.draw()
            img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
            cv2.imwrite(os.path.join(dir_sav+'/fig/' + filename + '_mass.png'),img)

            #cnormf = matplotlib.colors.Normalize(vmin=min(frics),vmax=max(frics))
            cnormf = matplotlib.colors.Normalize(vmin=0.1,vmax=1)
            for info in cellinfo:
                info['color'] = matplotlib.cm.coolwarm(cnormf(info['fric']))
            sim.setCellBodyProperties(obj,cellinfo)
            img = sim.draw()
            img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
            cv2.imwrite(os.path.join(dir_sav+'/fig/' + filename + '_fric.png'),img)
            