import os, sys
import argparse
import pickle
import yaml
import glob
import cv2
import matplotlib.pyplot as plt
import numpy as np

from DiffPhysics import DiffPhysics
#from lcp_physics_wrapper import MDiffPhysics
from OtherPhysics import *

from setups_sim import get_setups_tools_sim


dir_root = os.path.abspath('..')
if os.path.abspath(dir_root) not in sys.path:
    sys.path.append(dir_root)
from simulator import SimBullet

if __name__=='__main__':

    parser = argparse.ArgumentParser(description='Test')
    parser.add_argument('-i','--indir', type=str,  default='',     help='input expr file')
    parser.add_argument('-o','--outdir',type=str,  default='./res',help='output directory')
    parser.add_argument('-v','--video', type=str,  default='',    help='output video')
    parser.add_argument('-p','--param', type=int,  default=-1,    help='idx params')
    parser.add_argument('-m','--method',type=str,  required=True, help='method')
    parser.add_argument('--objects',    type=str,  nargs='+',     help='objects')
    parser.add_argument('--n_iters',    type=int,  default=50,    help='n_iters')
    parser.add_argument('--debug',      type=bool, default=False, help='visualization')
    args = parser.parse_args()

    if len(args.video) > 0 and not os.path.isdir(os.path.dirname(args.video)):
        print("[Error] Invalid path: ", args.video)
        exit(0)

    if args.method not in ['mdiff_cell',  'rand_cell', 'grand_cell', 'bf_cell', 'grid_cell', 'fdiff_cell', 'diff_cell',  'cma_cell',  'nm_cell',
                           'mdiff_group', 'rand_group','grand_group','bf_group','grid_group','fdiff_group','diff_group', 'cma_group', 'nm_group']:
        print('[Error] Invalid method: ', args.method)
        exit(0)

    if args.indir=='' and args.param<=0:
        print('[Error] set param (for sim) or set indir directory (for real)')
        exit(0)

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    def get_engine(method):
        if args.method.startswith('rand'):
            engine = RandomSearchPhysics(is_dbg=args.debug, fp_video=args.video)
        elif args.method.startswith('grand'):
            engine = GaussianRandomPhysics(is_dbg=args.debug, fp_video=args.video)
        elif args.method.startswith('bf'):
            engine = BruteForceSearchPhysics(is_dbg=args.debug, fp_video=args.video)
        elif args.method.startswith('grid'):
            engine = GridSearchPhysics(is_dbg=args.debug, fp_video=args.video)   
        elif args.method.startswith('fdiff'):
            engine = FiniteDiffPhysics(is_dbg=args.debug, fp_video=args.video)
        elif args.method.startswith('diff'):
            engine = DiffPhysics(is_dbg=args.debug, fp_video=args.video)
        elif args.method.startswith('mdiff'):
            engine = MDiffPhysics(is_dbg=args.debug, fp_video=args.video)
        elif args.method.startswith('cma'):
            engine = CMAPhysics(is_dbg=args.debug, fp_video=args.video)
        elif args.method.startswith('nm'):
            engine = NelderMeadPhysics(is_dbg=args.debug, fp_video=args.video)
        else:
            print('[Error] Invalid method: ', method)
            exit(0)

        return engine        

    # simulation
    if args.param > 0:
        if args.objects is None or len(args.objects)==0:
            args.objects = ['book','box','cramp','crimp','hammer','ranch','snack','toothpaste','windex']
        
        if args.method.endswith('group'):
            n_groups = [3,10,20]
        else:
            n_groups = [-1]

        times_sims = []
        times_grads = []

        for obj in args.objects:
            for n_trains in [5]:#range(5,0,-1):
                for n_group in n_groups:

                    print('%s - # of train: %d' % (obj,n_trains))

                    idx_param = args.param - 1
                    idxes_train = range(n_trains)
                    #idxes_test = [i for i in range(10) if i not in idxes_train]
                    idxes_test = range(1)
                    setups = get_setups_tools_sim([obj],idxes_train,idxes_test,idx_param)

                    engine = get_engine(args.method)
                               
                    if args.method.endswith('_cell'):                    
                        engine.setup(setups, n_group=n_group)
                        history = engine.infer(infer_type='cell_mass',nsimslimit=50)
                        n_group = len(engine.meta_targets[obj]['cells']['cells'])
                    else:                    
                        engine.setup(setups, n_group=n_group)
                        history = engine.infer(infer_type='group_mass')

                    print('Test Result: %.3f (m) %.3f (deg) %.3f (cell)' % 
                           (history[-1]['test_error_xy'],
                            history[-1]['test_error_yaw'],
                            history[-1]['test_error_cell']) )

                    summary = {'object':obj,
                               'idxes_train':idxes_train,
                               'idxes_test':idxes_test,
                               'history':history,
                              }

                    fp_save = os.path.join(args.outdir,
                                           '%s_%s_param%d_train%d_c%d.pkl'%\
                                           (args.method,obj,idx_param,len(idxes_train),n_group))
                    with open(fp_save,'wb') as f:
                        pickle.dump(summary, f, protocol=2)

                    times_sims.append( history[-1]['time_sims']/float(history[-1]['n_sims']) )
                    times_grads.append(history[-1]['time_grads']/float(history[-1]['n_grads']))
    
        print('average time / a simulation: {} (\pm{})'.format(np.mean(times_sims),  np.std(times_sims)))
        print('average time / a gradient: {} (\pm{})'.format(  np.mean(times_grads), np.std(times_grads)))

    # real experiment
    else:

        if args.objects is None or len(args.objects)==0:
            args.objects = ['book','box','cramp','crimp','hammer','ranch','snack','toothpaste','windex']

        for obj in args.objects:

            if obj=='snack':
                idxes_train = [1,3,5,7,9,13]
                idxes_test  = [0,2,6,8,10,12]
            else:
                idxes_train = [1,3,5,7,9,11,13]
                idxes_test  = [0,2,4,6,8,10,12]

            fp_label = os.path.join('../../dataset/objects',obj+'_label.pgm')
            setup = {'actor':{'name':'finger','type':'finger'},
                     'target_infos':[{'name':obj,'label':fp_label,'masses':[1,1,1]}],
                     'mass_minmax':[0.1, 5.0],
                     'train':idxes_train,
                     'test':idxes_test,
                     'real':[]}

            files = glob.glob(os.path.join(args.indir, obj+'_*_traj.txt'))
            files = sorted(files)

            for file in files:

                f = open(file,'r')
                node = yaml.load(f, Loader=yaml.BaseLoader)
                f.close()

                vec_x = float(node['push']['vec']['x'])
                vec_x = vec_x/abs(vec_x)

                push_x = float(node['push']['x']) - float(node['begin']['center_of_pixels']['x'])
                push_y = float(node['push']['y']) - float(node['begin']['center_of_pixels']['y'])

                x_res = float(node['transform']['x'])
                y_res = float(node['transform']['y'])                
                yaw_res = float(node['transform']['yaw'])
                
                action = {'finger':{'pos': [push_x,push_y], 'yaw':0,
                                    'velocity':[float(node['push']['vec']['x']),
                                                float(node['push']['vec']['y']),
                                                float(node['push']['vec']['z'])],
                                    'duration':1.0},
                          'targets':[obj], 
                          obj:{'pos':[0,0], 'yaw':0, 
                               'pos_res':[x_res,y_res], 'yaw_res':yaw_res}
                         }
                setup['real'].append(action)

                if True:
                    print(file)
                    print('push_x:',push_x)
                    print('push_y:',push_y)

                    sim = SimBullet(gui=False)
                    sim._p.setGravity(0,0,-9.8)
                    sim.setupRendering(0.75,-90,-89.99,500,500)
                    sim.addObject('finger0','finger',[1,1,0,1])
                    sim.addObject('finger1','finger',[1,0,1,1])
                    sim.addObject('table_rect','table_rect',[1,1,1,1])
                    img_label = cv2.imread(fp_label, cv2.IMREAD_UNCHANGED)
                    cells = CellPhysicsBase.createCells(img_label, [0,0,0,0], meter_per_pixel=0.53/640.0)
                    sim.addObject(obj+'0',cells, [0,0,1,1])
                    sim.addObject(obj+'1',cells, [1,0,0,1])
                    sim.setObjectPose('finger0', push_x,     push_y, 0)
                    sim.setObjectPose('finger1', push_x+float(node['push']['vec']['x']), push_y, 0)
                    sim.setObjectPose(obj+'0', 0, 0, 0)
                    sim.setObjectPose(obj+'1', x_res, y_res, yaw_res)

                    draw = sim.draw()
                    plt.imshow(draw)
                    plt.show()

            if True:
                engine = get_engine(args.method)
                engine.setup(setup, n_group=-1)
                n_group = len(engine.meta_targets[obj]['cells']['cells'])

                history = engine.infer(infer_type='cell_mass',nsimslimit=100)

                print('[Test Result] cell err: %.3f (m) pose err: %.3f (m) %.3f (deg)' % 
                       (history[-1]['test_error_cell'],
                        history[-1]['test_error_xy'],
                        history[-1]['test_error_yaw']) )

                summary = {'object':obj,
                           'idxes_train':idxes_train,
                           'idxes_test':idxes_test,
                           'history':history,
                          }

                fp_save = os.path.join(args.outdir,
                                       '%s_%s_real_set%d_c%d.pkl'%\
                                       (args.method,obj,-1,n_group))
                with open(fp_save,'wb') as f:
                    pickle.dump(summary, f, protocol=2)