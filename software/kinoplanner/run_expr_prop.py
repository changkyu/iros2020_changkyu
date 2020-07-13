import os, sys
import numpy as np
import cv2

dir_root = os.path.abspath('..')
if os.path.abspath(dir_root) not in sys.path:
    sys.path.append(dir_root)
from simulator import SimBullet
from simulator import distance_angle
from cellphysics import CellPhysicsBase

from KinoPlanner import KinoPlanner
from ProbPhysicsPlanner import ProbPhysicsPlanner

import glob
import pickle
import argparse

rnds_x = \
[0.28653127, 0.13155485, 0.96229978, 0.74852533, 0.14675884,
 0.18652149, 0.9930919 , 0.44366939, 0.90339929, 0.70027604,
 0.06296849, 0.12488595, 0.63201549, 0.71744801, 0.47109163,
 0.94178824, 0.12861877, 0.58718975, 0.03193234, 0.81925039,
 0.94076377, 0.34777335, 0.37851971, 0.08537728, 0.11305514,
 0.70069913, 0.22640526, 0.59425543, 0.99472694, 0.12062213,
 0.69934227, 0.53170905, 0.62747592, 0.90765082, 0.81888995,
 0.58143064, 0.03322373, 0.08992933, 0.05102894, 0.15914133,
 0.75106285, 0.1964225 , 0.28709339, 0.31571282, 0.7985778 ,
 0.04673159, 0.54760356, 0.50343577, 0.28840341, 0.29092593,
 0.76416441, 0.64895179, 0.52516504, 0.50011101, 0.251998  ,
 0.27137476, 0.03815355, 0.56874577, 0.1884927 , 0.55200782,
 0.64293081, 0.33215305, 0.07128928, 0.19790485, 0.72261648,
 0.20606281, 0.18637498, 0.8674672 , 0.04341219, 0.58227806,
 0.11700094, 0.18566013, 0.64435721, 0.51001409, 0.58397873,
 0.27753224, 0.62321015, 0.4845986 , 0.60101164, 0.37990118,
 0.40578318, 0.23231402, 0.39974131, 0.44714439, 0.16273886,
 0.09214673, 0.94537657, 0.47886814, 0.0038613 , 0.71399769,
 0.19226771, 0.47598252, 0.01536853, 0.49127696, 0.51900822,
 0.38823259, 0.01818152, 0.55736644, 0.95826322, 0.35858202]

rnds_y = \
[0.10721016, 0.19412393, 0.08963375, 0.73020267, 0.7433705 ,
 0.8101552 , 0.78701632, 0.11288006, 0.82993897, 0.94741494,
 0.71434683, 0.68125819, 0.38418047, 0.47100194, 0.0309117 ,
 0.12066643, 0.5285642 , 0.03391164, 0.88628993, 0.17680766,
 0.99068546, 0.49451337, 0.50954802, 0.9294948 , 0.36438615,
 0.31305342, 0.89279417, 0.20311246, 0.09258377, 0.45946872,
 0.79755524, 0.99999429, 0.117515  , 0.57985059, 0.14493412,
 0.31422818, 0.13626561, 0.08511088, 0.30381498, 0.1419733 ,
 0.27480295, 0.88684868, 0.16890424, 0.4203718 , 0.65946469,
 0.08600826, 0.82403132, 0.14240164, 0.4024938 , 0.658349  ,
 0.21672774, 0.28593005, 0.01952989, 0.65564653, 0.7434169 ,
 0.85000066, 0.70840309, 0.9177029 , 0.37496943, 0.0295357 ,
 0.72521065, 0.34619669, 0.32742908, 0.95086639, 0.5337815 ,
 0.68653475, 0.47973183, 0.00380566, 0.16701777, 0.1736657 ,
 0.12813417, 0.88774639, 0.65344985, 0.23211935, 0.58259425,
 0.32420653, 0.50812263, 0.68525285, 0.18961533, 0.94658914,
 0.78193136, 0.39748956, 0.15476869, 0.39521841, 0.55024066,
 0.21417718, 0.96574716, 0.9944274 , 0.71973274, 0.12677393,
 0.0726414 , 0.25293111, 0.58384647, 0.59479484, 0.26459815,
 0.26901791, 0.29718839, 0.03263161, 0.5535058 , 0.28296895]

rnds_yaw = \
[0.150042  , 0.59449011, 0.95222735, 0.32704214, 0.48270676,
 0.48351668, 0.73348208, 0.10590432, 0.64169109, 0.53033394,
 0.26380189, 0.87374503, 0.97952779, 0.27469725, 0.34862411,
 0.14230171, 0.28155993, 0.64195246, 0.70770892, 0.49376498,
 0.84423786, 0.15730628, 0.38593106, 0.36991003, 0.63182125,
 0.15102012, 0.64178297, 0.23100422, 0.29520773, 0.39562066,
 0.83335818, 0.70151747, 0.76746158, 0.41553082, 0.19983076,
 0.31604711, 0.78486798, 0.50774221, 0.98465928, 0.98849895,
 0.07027289, 0.06486826, 0.15557687, 0.1878066 , 0.12515278,
 0.29117524, 0.71171032, 0.34331462, 0.40133836, 0.11696835,
 0.31591903, 0.64365822, 0.70960418, 0.313238  , 0.19549951,
 0.9452164 , 0.27236005, 0.45865058, 0.14042791, 0.49708467,
 0.16398568, 0.59395185, 0.77512995, 0.50071204, 0.43673978,
 0.98500384, 0.01222329, 0.00557318, 0.28389736, 0.38487479,
 0.13843793, 0.35482043, 0.34327212, 0.61674046, 0.30130758,
 0.14219809, 0.26728923, 0.87518891, 0.83353954, 0.97343613,
 0.74514342, 0.54575302, 0.86652752, 0.34261425, 0.02001768,
 0.00123449, 0.76772096, 0.57393085, 0.18097536, 0.3434161 ,
 0.25379904, 0.16999243, 0.76474458, 0.84527379, 0.63356761,
 0.76047061, 0.48736997, 0.35686898, 0.72443655, 0.11784546]

"""
gts = \
{    
    'book':      {'masses':[1,4],   'goal':[0.63,-0.50,179/180.0*np.pi]},
    'box':       {'masses':[3,1],   'goal':[0.63,-0.50,000/180.0*np.pi]},
    'crimp':     {'masses':[3,1],   'goal':[0.63,-0.50,000/180.0*np.pi]},
    'hammer':    {'masses':[3,1],   'goal':[0.63,-0.50,000/180.0*np.pi]},
    'ranch':     {'masses':[3,1,2], 'goal':[0.63,-0.50,000/180.0*np.pi]},
    'snack':     {'masses':[1,3],   'goal':[0.63,-0.49,179/180.0*np.pi]},
    'toothpaste':{'masses':[3,1],   'goal':[0.63,-0.50,000/180.0*np.pi]},
    'windex':    {'masses':[1,3],   'goal':[0.63,-0.50,179/180.0*np.pi]},
}
"""

if __name__=='__main__':

    meter_per_pixel = 0.53/640.0
    objects = ['book','box','hammer','ranch','snack','windex']
    methods = ['known','prop','uniform','none','det'] + ['det'+str(i) for i in range(1,10)]
    exprs = ['round','tight']
    dir_sav = '../cellphysics/sav/'

    parser = argparse.ArgumentParser(description='Test')
    parser.add_argument('-o','--outdir',type=str,  default='./res_prop',help='output directory')
    parser.add_argument('--skip',       type=bool, default=False, help='skip')
    parser.add_argument('--method',     type=str,  nargs='+',     help='method',  choices=methods)
    parser.add_argument('--expr',       type=str,  default='round',help='experiment (round, tight)',  choices=exprs)
    parser.add_argument('--objects',    type=str,  nargs='+',     help='objects')
    parser.add_argument('--sets',       type=int,  default=-1,    help='sets')
    parser.add_argument('--inits',      type=int,  nargs='+',     help='inits')
    parser.add_argument('--debug',      type=bool, default=False, help='visualization')
    args = parser.parse_args()

    if args.expr!='round' and args.outdir=='./res_prop':
        args.outdir = args.outdir + '_' + args.expr

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    if args.objects is None or len(args.objects)==0:
        args.objects = objects

    if args.method is None or len(args.method)==0:
        args.method = methods

    for method in args.method:
        if method not in methods:
            print('[Error] Invalid method: {} (choose from {})'.format(method,methods))
            exit(0)

    if args.expr=='tight':
        width=0.25
        setup = {'table_type':'table_tight'}
    else:
        width=0.50
        setup = {'table_type':'table_round'}

    gts = \
    {    
        'book':      {'masses':[0.36953162, 4.04311101],            'goal':[0.63,-width,      179/180.0*np.pi]},
        'box':       {'masses':[4.87099288, 1.50774383],            'goal':[0.63,-width,      000/180.0*np.pi]},
        'crimp':     {'masses':[4.13308188, 1.63447027],            'goal':[0.63,-width,      000/180.0*np.pi]},
        'hammer':    {'masses':[3.31954728, 0.74380948],            'goal':[0.63,-width,      000/180.0*np.pi]},
        'ranch':     {'masses':[3.9006551,  0.00511314, 1.61148054],'goal':[0.63,-width,      000/180.0*np.pi]},
        'snack':     {'masses':[0.93759184, 4.39030386],            'goal':[0.63,-width+0.01, 179/180.0*np.pi]},
        'toothpaste':{'masses':[3.24363832, 1.6426988 ],            'goal':[0.63,-width,      000/180.0*np.pi]},
        'windex':    {'masses':[0.1252993 , 4.24386566],            'goal':[0.63,-width,      179/180.0*np.pi]},
    }
   
    inits = []
    for rnd_x, rnd_y, rnd_yaw in zip(rnds_x,rnds_y,rnds_yaw):
        if args.expr=='round':
            x = 0.63
            y = 0
        else:
            x = 0.5*(rnd_x-0.5) + 0.63
            y = 0.5*(rnd_y-0.5)

        yaw = np.pi*(rnd_yaw-0.5)*2
        inits.append((x,y,yaw))

    if args.sets > 0:
        args.inits = range((args.sets-1)*10,args.sets*10)
    elif args.inits is None or len(args.inits)==0:
        args.inits = range(len(inits))

    if 'det' in args.method:
        args.method.remove('det')
        args.method = args.method + ['det'+str(i) for i in range(1,10)]
    print(args.method)

    for obj in args.objects:
        fp_label = '../../dataset/objects/%s_label.pgm' % obj

        goals = []
        goal = list(gts[obj]['goal'])
        goal[2] = 0            
        goals.append(list(goal))
        goal[2] = 0.99*np.pi
        goals.append(list(goal))
                
        for method in args.method:
            models = []
            if method=='known':                
                models.append({'cellinfo':gts[obj]['masses'],'error_cell':0})
                n_plans = 1
            elif method=='prop' or method.startswith('det'):
                if method=='prop':
                    files = glob.glob(dir_sav+'/'+obj+'*.pkl')
                else:
                    files = glob.glob(dir_sav+'/'+obj+'_seed'+method[3:]+'.pkl')
                    print(dir_sav+'/'+obj+'_seed'+method[3:]+'.pkl')

                for file in files:
                    print('load ' + file)
                    f = open(file,'rb')
                    models.append(pickle.load(f))
                    f.close()
                n_plans = 10
            elif method=='uniform' or method=='none':
                models.append({'cellinfo':[1,1,1,1],'error_cell':0})
                if method=='uniform':
                    n_plans = 10
                elif method=='none':
                    n_plans = 1

            # create planner
            planner = ProbPhysicsPlanner(obj,fp_label,models,meter_per_pixel,
                                         setup=setup,
                                         contact_resolution=0.02, \
                                         pushing_dists=[0.20,0.10,0.05,0.04,0.03,0.02,0.01])

            # select goals
            goals_sel = planner.selectGoals(obj, goals)
            print('Selected Goals: ', goals_sel)

            sim = SimBullet(gui=args.debug)
            sim.setupCamera(0.75,-90,-89.99, 0.63,0,0)
            sim.addObject('table_round','table_round',[1,1,1,1])
            sim.setObjectPose('table_round',0.63,0,0)
            sim.addObject('finger','finger',[1,1,0,1])
            sim.setObjectPose('finger',10,0,0)
            cells = CellPhysicsBase.createCells(fp_label,gts[obj]['masses'],meter_per_pixel=meter_per_pixel)
            sim.addObject(obj, cells, [0.5,0.5,0.5,1], 0, 0, 0)

            succs = []
            plans = []
            n_actions = []
            for i in args.inits:
                init = inits[i]
                print('method:%s obj:%s init#:%d'%(method,obj,i))

                fp_save = os.path.join(args.outdir,'%s_%s_init%d.pkl'%(method,obj,i))
                if args.skip and os.path.isfile(fp_save):
                    print('[SKIP] %s' % fp_save)
                    with open(fp_save,'rb') as f:
                        tmp = pickle.load(f)
                        succs.append(tmp['success'])
                        plans.append(tmp['plan'])
                        n_actions.append(tmp['plan_length'])
                    continue

                sim.setObjectPose('finger',10,0,0)
                sim.setObjectPose(obj,init[0],init[1],init[2])

                dist_yaw_min = np.inf
                for goal_sel in goals_sel:
                    dist_yaw = abs(distance_angle(goal_sel[2],init[2]))
                    if dist_yaw_min > dist_yaw:
                        dist_yaw_min = dist_yaw
                        goal = goal_sel

                path = [init]
                path_planned = [init]
                actions = []
                actions_rejected = []
                succ = False
                while True:

                    x0,y0,yaw0 = sim.getObjectPose(obj)
                    cellpos0 = sim.getCellBodyPose(obj)

                    plans = planner.plan((x0,y0,yaw0),goal, thres=0.02, n_plans=n_plans, actions_rejected=actions_rejected)
                    action_res = plans[0]['action']

                    KinoPlanner.applyAction(sim, obj, x0,y0,yaw0, action_res, 'finger',sim.shapes['finger']['dims'])

                    x1,y1,yaw1 = sim.getObjectPose(obj)
                    cellpos1 = sim.getCellBodyPose(obj)
                    err_cell = CellPhysicsBase.distanceCellPos(cellpos0, cellpos1)

                    path.append((x1,y1,yaw1))
                    path_planned.append(plans[0]['nexts'][0][1:4])
                    actions.append(action_res)

                    (_,_,z1),_ = sim.getObjectPose(obj,ndims=3)
                    if z1<0:
                        print("Failed")
                        succs.append(False)
                        succ = False
                        break

                    if err_cell <= 0.005:
                        actions_rejected.append(action_res)
                    else:
                        actions_rejected = []

                    err_xy  = np.linalg.norm([x1-goal[0],y1-goal[1]])
                    err_yaw = abs(distance_angle(yaw1,goal[2]))

                    print('[%d] err_xy: %.3f err_yaw: %.3f' %(len(actions),err_xy,err_yaw/np.pi*180))

                    success = (err_xy <= 0.02) and (err_yaw <= 15.0/180.0*np.pi)
                    if success:
                        print("SUCCESS")
                        succs.append(True)
                        succ = True
                        break

                n_actions.append(len(actions))
                plan = {'path':path, 'path_expected':path_planned, 'actions':actions}
                summary = {'success':succ, 'plan':plan, 'plan_length':len(actions)}                
                with open(fp_save,'wb') as f:
                    pickle.dump(summary, f, protocol=2)

                print('current success rate: %.2f%% (%d/%d), plan length: %.2f' % \
                      (sum(succs)*100/float(len(succs)),sum(succs),len(succs),\
                       sum(n_actions)/float(len(n_actions))))


