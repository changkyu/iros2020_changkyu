import os, sys
import pickle
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import cv2
import pybullet as p

dir_pusher = os.path.abspath('..')
if os.path.abspath(dir_pusher) not in sys.path:
    sys.path.append(dir_pusher)
from simulator import SimBullet

from PhysicsBase import CellPhysicsBase
from setups_sim import get_setups_tools_sim
from setups_sim import params

def pixel2meter(cells, action):
    x_px = action['finger']['pos_px'][0]
    y_px = action['finger']['pos_px'][1]
    x = -y_px * cells['meter_per_pixel'] + cells['offset'][0]
    y = -x_px * cells['meter_per_pixel'] + cells['offset'][1]
    action['finger']['pos'] = [x,y]


def draw_result(fp_label, fp_res, fp_massmap, fp_posres, setups, masses_gt=[1,1,1,1], iters=[-1,0,1,2,]):
    
    cmap = matplotlib.cm.coolwarm
    #cnorm = matplotlib.colors.Normalize(vmin=0.1,vmax=4.9)
    
    obj = os.path.split(fp_label)[1].split('_')[0]

    sim = SimBullet(gui=False)
    #sim._p.setGravity(0,0,-9.8)
    sim.addObject('table_rect','table_rect',[1,1,1,0])
    sim.setupRendering(0.75,-90,-60,500,500)
    img_label = cv2.imread(fp_label, cv2.IMREAD_UNCHANGED)
    labels = np.unique(img_label)
    cells = CellPhysicsBase.createCells(img_label, masses_gt, 0.53/640.0)
    sim.addObject(obj,cells, [0,0,1,1])
    sim.setObjectPose(obj, 0, 0, 0)
    
    cellinfos = sim.getCellBodyProperties(obj)
    vmax = max([info['mass'] for info in cellinfos])
    vmin = min([info['mass'] for info in cellinfos])
    cnorm = matplotlib.colors.Normalize(vmin=vmin,vmax=vmax)
    for info in cellinfos:
        info['color'] = cmap(cnorm(info['mass']))
    sim.setCellBodyProperties(obj,cellinfos)
    img = sim.draw()
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    cv2.imwrite(fp_massmap % -3,img)

    f = open(fp_res,'rb')
    res = pickle.load(f)
    f.close()

    # estimation example
    cellinfos = res['history'][-1]['cellinfos']
    #cnorm = matplotlib.colors.Normalize(vmin=0.1,vmax=5)
    for info in cellinfos[obj]:
        info['color'] = cmap(cnorm((vmax-vmin)*0.5))
    sim.setCellBodyProperties(obj,cellinfos[obj])
    img = sim.draw()

    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    cv2.imwrite(fp_massmap % -2,img)
    img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)

    for it in iters:

        if it >= len(res['history']):
            continue

        cellinfos = res['history'][it]['cellinfos']
        #vmax = max([info['mass'] for info in cellinfos[obj]])
        #vmin = min([info['mass'] for info in cellinfos[obj]])
        #cnorm = matplotlib.colors.Normalize(vmin=vmin,vmax=vmax)
        for info in cellinfos[obj]:
            info['color'] = cmap(cnorm(info['mass']))
        sim.setCellBodyProperties(obj,cellinfos[obj])
        img = sim.draw()
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        cv2.imwrite(fp_massmap % it,img)
        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)

    return

    # debug image
    sim.setupRendering(0.75,90,-90,500,500)

    sim.addObject('finger1','finger',[1,1,0.9,1])
    sim.addObject('finger2','finger',[1,1,0.6,1])
    sim.addObject('finger3','finger',[1,1,0.3,1])

    sim.addObject('init',cells, [0.7,0.7,0.7,1])
    pos, rot = sim.getPosRot(sim.objects['init']['shape'], 0,0,0)
    pos[2] = pos[2] - 0.002
    sim._p.resetBasePositionAndOrientation(sim.objects['init']['bodyId'],pos,rot)

    sim.addObject('gt',cells, [0,1,0,1])
    for i in range(len(res['history'][-1]['test_poses'])):

        if 'real' not in setups:
            pixel2meter(cells,setups['test'][i])
            sim.resetObject('finger1',setups['test'][i]['finger']['pos'][0] - 0.03,\
                                      setups['test'][i]['finger']['pos'][1],0)
            sim.resetObject('finger2',setups['test'][i]['finger']['pos'][0] + setups['test'][i]['finger']['velocity'][0]*0.40 - 0.03,\
                                      setups['test'][i]['finger']['pos'][1] + setups['test'][i]['finger']['velocity'][1]*0.40,0)
            sim.resetObject('finger3',setups['test'][i]['finger']['pos'][0] + setups['test'][i]['finger']['velocity'][0]*0.90 - 0.03,\
                                      setups['test'][i]['finger']['pos'][1] + setups['test'][i]['finger']['velocity'][1]*0.90,0)
        else:
            sim.resetObject('finger1',setups['real'][i]['finger']['pos'][0] - 0.03,\
                                      setups['real'][i]['finger']['pos'][1],0)
            sim.resetObject('finger2',setups['real'][i]['finger']['pos'][0] + setups['real'][i]['finger']['velocity'][0]*0.40 - 0.03,\
                                      setups['real'][i]['finger']['pos'][1] + setups['real'][i]['finger']['velocity'][1]*0.40,0)
            sim.resetObject('finger3',setups['real'][i]['finger']['pos'][0] + setups['real'][i]['finger']['velocity'][0]*0.90 - 0.03,\
                                      setups['real'][i]['finger']['pos'][1] + setups['real'][i]['finger']['velocity'][1]*0.90,0)
        
        pose_gt = res['history'][-1]['test_poses'][i]['gt']

        #sim.resetObject(obj,100,100,0)
        sim.resetObject('gt',pose_gt[obj]['x'], pose_gt[obj]['y'], pose_gt[obj]['yaw'])
        #img = sim.draw()
        #img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        #cv2.imwrite(fp_posres % (i+20),img)
        #img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)


        #sim.resetObject('gt',100,100,0)
        pose_est = res['history'][-1]['test_poses'][i]['estimation']
        sim.resetObject( obj, pose_est[obj]['x'],
                              pose_est[obj]['y'],
                              pose_est[obj]['yaw'] )
        
        pos, rot = sim.getPosRot(sim.objects['gt']['shape'], pose_gt[obj]['x'], pose_gt[obj]['y'], pose_gt[obj]['yaw'])
        pos[0] = pos[0] - 0.001
        pos[2] = pos[2] - 0.001
        sim._p.resetBasePositionAndOrientation(sim.objects['gt']['bodyId'],pos,rot)

        img = sim.draw()        
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        cv2.imwrite(fp_posres % i,img)
        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)


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

prefix = 'sims'
#prefix = 'real'

if prefix=='sims':
    obj2ncell = {'book':  28,
                 'box':   65,
                 'cramp': 86,
                 'crimp': 44,
                 'hammer':88,
                 'ranch': 52,
                 'snack': 64,
                 'toothpaste': 74,
                 'windex': 84}
    #objects = obj2ncell.keys()
    objects = ['book','box','crimp','hammer','ranch','snack','toothpaste','windex']
else:
    obj2ncell = {'book':  28,
                 'box':   65,
                 'cramp': 86,
                 'crimp': 44,
                 'hammer':88,
                 'ranch': 52,
                 'snack': 64,
                 'toothpaste': 74,
                 'windex': 84}
    #objects = ['hammer','cramp','ranch','crimp']
    objects = ['book','box','hammer','snack','windex']

if __name__=='__main__':

    if not os.path.isdir('fig/draw'):
        os.makedirs('fig/draw')

    obj = 'hammer'
    obj = 'ranch'
    obj = 'crimp'
    obj = 'cramp'
    idx_param = 0
    n_train = 1

    imgs = []

    if False:

        obj = 'box'

        fp_label   = '../../pusher/data/real/ver5/'+obj+'_push1_begin_label.png'
        fp_setup   = '../../pusher/data/real/ver5/'+obj+'_push_all.pkl'

        fp_res     = './res/diff_cell_'+obj+'_real_set-1_c96.pkl'
        fp_massmap = './fig/draw/draw_diff_cell_'+obj+'_real_massmap_iter%d.png'
        fp_posres  = './fig/draw/draw_diff_cell_'+obj+'_real_posres%d.png'
        
        f = open(fp_setup,'rb')
        setup = pickle.load(f)
        f.close()

        draw_result(fp_label, fp_res, fp_massmap, fp_posres, setup)

    else:
        for obj in ['toothpaste']:#['book','box','crimp','hammer','ranch','snack','toothpaste','windex']:
            for idx_param in range(0,10):
                for n_train in range(1,6):

                    #fp_res = './res/diff_cell_%s_param%d_train%d_c110.pkl' % (obj,idx_param,n_train)
                    #fp_res = '../../results/prop/sims/diff/diff_cell_%s_param%d_train%d_set%d_c%d.pkl' % (obj,idx_param,n_train,obj2sets[prefix][obj][0], obj2ncell[obj])
                    fp_res = 'res_tmp/diff/diff_cell_%s_param%d_train%d_set%d_c%d.pkl' % (obj,idx_param,n_train,obj2sets[prefix][obj][0], obj2ncell[obj])
                    fp_label = '../../dataset/objects/%s_label.pgm' % obj

                    if not os.path.isfile(fp_res):
                        print('[Error] Invalid file: ' + fp_res)
                        exit(0)
                    if not os.path.isfile(fp_label):
                        print('[Error] Invalid file: ' + fp_label)
                        exit(0)
                    
                    fp_massmap = 'fig/draw/draw_diff_cell_%s_param%d_train%d_iter%%d.png' % (obj,idx_param,n_train)
                    fp_posres =  'fig/draw/debug_diff_cell_%s_param%d_train%d_test%%d.png' % (obj,idx_param,n_train)
                    
                    idxes_train = [n_train-1]
                    idxes_test = range(12)
                    setups = get_setups_tools_sim([obj],idxes_train,idxes_test,idx_param)

                    draw_result(fp_label, fp_res, fp_massmap, fp_posres, setups, params[obj][idx_param]['masses'])
    #fig = plt.figure()
    #ax = fig.add_subplot(111)
    #print(np.max(imgs[0]/2))
    #ax.imshow(imgs[0]//2 + imgs[1]//2)
    #ax.imshow(imgs[1])
    #cb = plt.colorbar(mappable=matplotlib.cm.ScalarMappable(norm=cnorm, cmap=cmap), ax=ax)    
    #plt.show()