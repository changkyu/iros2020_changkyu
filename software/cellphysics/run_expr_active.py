import os, sys
import argparse
import pickle
import glob
import yaml

from ActiveDiffPhysics import ActiveDiffPhysics
#from lcp_physics_wrapper import MDiffPhysics
from OtherPhysics import *

from setups_sim import get_setups_tools_sim
idxes_sim = [
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
[[[0, 4], [0, 1, 3, 4, 5, 7, 8, 9]],
 [[1, 9], [0, 1, 2, 3, 4, 7, 8, 9]],
 [[2, 8], [0, 1, 2, 3, 4, 5, 7, 8]],
 [[3, 9], [0, 1, 2, 3, 4, 5, 6, 7]],
 [[4, 9], [0, 1, 2, 5, 6, 7, 8, 9]],
 [[5, 4], [2, 3, 4, 5, 6, 7, 8, 9]],
 [[6, 8], [1, 2, 3, 4, 5, 6, 8, 9]],
 [[7, 2], [0, 1, 2, 3, 5, 6, 7, 8]],
 [[8, 3], [0, 1, 2, 3, 4, 7, 8, 9]],
 [[9, 5], [1, 2, 4, 5, 6, 7, 8, 9]]],
[[[0, 4, 9], [0, 1, 2, 3, 5, 7, 8]],
 [[1, 9, 3], [0, 1, 2, 4, 6, 7, 8]],
 [[2, 8, 5], [0, 1, 2, 3, 4, 7, 9]],
 [[3, 9, 8], [0, 1, 2, 3, 5, 6, 7]],
 [[4, 9, 6], [0, 1, 2, 4, 5, 7, 8]],
 [[5, 4, 6], [0, 1, 2, 5, 7, 8, 9]],
 [[6, 8, 1], [0, 2, 3, 4, 5, 6, 9]],
 [[7, 2, 3], [0, 1, 4, 5, 6, 8, 9]],
 [[8, 3, 0], [1, 2, 5, 6, 7, 8, 9]],
 [[9, 5, 8], [1, 2, 3, 4, 6, 7, 9]]],
[[[0, 6, 7, 2], [1, 3, 4, 5, 8, 9]],
 [[1, 4, 8, 6], [1, 2, 3, 5, 7, 9]],
 [[2, 7, 5, 8], [0, 1, 2, 3, 4, 9]],
 [[3, 4, 2, 8], [0, 1, 3, 5, 6, 9]],
 [[4, 8, 0, 5], [0, 1, 2, 3, 7, 9]],
 [[5, 8, 9, 2], [0, 1, 3, 4, 6, 8]],
 [[6, 1, 7, 8], [0, 2, 3, 5, 6, 9]],
 [[7, 1, 2, 4], [0, 3, 5, 6, 7, 9]],
 [[8, 4, 1, 7], [0, 3, 5, 6, 8, 9]],
 [[9, 0, 4, 3], [1, 2, 5, 6, 8, 9]]],
[[[0, 6, 8, 5, 9], [0, 2, 3, 4, 7]],
 [[1, 6, 5, 4, 2], [0, 3, 5, 7, 8]],
 [[2, 9, 8, 0, 7], [1, 2, 3, 4, 5]],
 [[3, 4, 8, 5, 0], [1, 2, 3, 7, 9]],
 [[4, 2, 3, 8, 6], [0, 4, 5, 7, 9]],
 [[5, 2, 1, 4, 6], [0, 3, 6, 7, 9]],
 [[6, 5, 9, 7, 3], [0, 1, 2, 4, 6]],
 [[7, 4, 2, 6, 0], [3, 5, 7, 8, 9]],
 [[8, 7, 6, 4, 3], [0, 1, 5, 8, 9]],
 [[9, 6, 9, 5, 7], [0, 1, 3, 4, 8]]],
]

"""
idxes_real = [
[[[0],[]],
 [[1],[]],
 [[2],[]],
 [[3],[]],
 [[4],[]],
 [[5],[]],
 [[6],[]]],
[[[0, 1],[]],
 [[1, 2],[]],
 [[2, 0],[]],
 [[3, 1],[]],
 [[4, 5],[]],
 [[5, 4],[]],
 [[6, 5],[]]],
[[[0, 1, 2],[]],
 [[1, 2, 0],[]],
 [[2, 0, 6],[]],
 [[3, 1, 4],[]],
 [[4, 5, 2],[]],
 [[5, 4, 1],[]],
 [[6, 5, 7],[]]],
[[[0, 1, 2, 6],[]],
 [[1, 2, 0, 4],[]],
 [[2, 0, 6, 1],[]],
 [[3, 1, 4, 5],[]],
 [[4, 5, 2, 1],[]],
 [[5, 4, 1, 0],[]],
 [[6, 5, 7, 2],[]]],
[[[0, 1, 2, 6, 4],[]],
 [[1, 2, 0, 4, 5],[]],
 [[2, 0, 6, 1, 3],[]],
 [[3, 1, 4, 5, 2],[]],
 [[4, 5, 2, 1, 6],[]],
 [[5, 4, 1, 0, 2],[]],
 [[6, 5, 7, 2, 8],[]]],
]
"""

"""
idxes_real = [
[[[0, ],[]],
 [[1, ],[]],
 [[2, ],[]],
 [[3, ],[]],
 [[4, ],[]],
 [[5, ],[]],
 [[6, ],[]],
 [[7, ],[]],
 [[8, ],[]],
 [[9, ],[]],
 [[10,],[]],
 [[11,],[]],
 [[12,],[]],
 [[13,],[]]],
[[[0, 12,],[]],
 [[1,  0,],[]],
 [[2, 10,],[]],
 [[3,  5,],[]],
 [[4,  3,],[]],
 [[5,  3,],[]],
 [[6,  0,],[]],
 [[7,  9,],[]],
 [[8,  1,],[]],
 [[9,  7,],[]],
 [[10, 3,],[]],
 [[11, 6,],[]],
 [[12, 3,],[]],
 [[13, 4,],[]]],
[[[0, 12, 10,],[]],
 [[1,  0,  2,],[]],
 [[2, 10, 12,],[]],
 [[3,  5,  7,],[]],
 [[4,  3,  2,],[]],
 [[5,  3, 12,],[]],
 [[6,  0,  2,],[]],
 [[7,  9,  3,],[]],
 [[8,  1, 11,],[]],
 [[9,  7, 10,],[]],
 [[10, 3, 12,],[]],
 [[11, 6, 12,],[]],
 [[12, 3,  5,],[]],
 [[13, 4, 11,],[]]],
[[[0, 12, 10,  2,],[]],
 [[1,  0,  2, 11,],[]],
 [[2, 10, 12,  3,],[]],
 [[3,  5,  7,  4,],[]],
 [[4,  3,  2,  5,],[]],
 [[5,  3, 12,  9,],[]],
 [[6,  0,  2, 11,],[]],
 [[7,  9,  3,  0,],[]],
 [[8,  1, 11,  7,],[]],
 [[9,  7, 10,  6,],[]],
 [[10, 3, 12,  4,],[]],
 [[11, 6, 12,  1,],[]],
 [[12, 3,  5, 13,],[]],
 [[13, 4, 11,  6,],[]]],
[[[0, 12, 10,  2,  5],[]],
 [[1,  0,  2, 11,  5],[]],
 [[2, 10, 12,  3,  4],[]],
 [[3,  5,  7,  4,  0],[]],
 [[4,  3,  2,  5,  6],[]],
 [[5,  3, 12,  9, 10],[]],
 [[6,  0,  2, 11,  7],[]],
 [[7,  9,  3,  0,  2],[]],
 [[8,  1, 11,  7,  2],[]],
 [[9,  7, 10,  6,  8],[]],
 [[10, 3, 12,  4,  0],[]],
 [[11, 6, 12,  1,  5],[]],
 [[12, 3,  5, 13,  9],[]],
 [[13, 4, 11,  6,  7],[]]],
]
"""

idxes_real = [
[[[0],[]],
 [[1],[]],
 [[2],[]],
 [[3],[]],
 [[4],[]],
 [[5],[]],
 [[6],[]]],
[[[0, 6],[]],
 [[1, 5],[]],
 [[2, 6],[]],
 [[3, 4],[]],
 [[4, 2],[]],
 [[5, 3],[]],
 [[6, 0],[]]],
[[[0, 6, 2],[]],
 [[1, 5, 0],[]],
 [[2, 6, 6],[]],
 [[3, 4, 4],[]],
 [[4, 2, 2],[]],
 [[5, 3, 1],[]],
 [[6, 0, 7],[]]],
[[[0, 6, 2, 6],[]],
 [[1, 5, 0, 4],[]],
 [[2, 6, 6, 1],[]],
 [[3, 4, 4, 5],[]],
 [[4, 2, 2, 1],[]],
 [[5, 3, 1, 0],[]],
 [[6, 0, 7, 2],[]]],
[[[0, 6, 2, 6, 4],[]],
 [[1, 5, 0, 4, 5],[]],
 [[2, 6, 6, 1, 3],[]],
 [[3, 4, 4, 5, 2],[]],
 [[4, 2, 2, 1, 6],[]],
 [[5, 3, 1, 0, 2],[]],
 [[6, 0, 7, 2, 8],[]]],
]

if __name__=='__main__':

    parser = argparse.ArgumentParser(description='Test')
    parser.add_argument('-i','--indir', type=str,  default='',     help='input expr file')
    parser.add_argument('-o','--outdir',type=str,  default='./res',help='output directory')
    parser.add_argument('-v','--video', type=str,  default='',    help='output video')
    parser.add_argument('-p','--param', type=int,  default=-1,    help='idx params')
    parser.add_argument('-m','--method',type=str,  required=True, help='method')
    parser.add_argument('--sets',       type=int,  default=-1,     help='set idxes')
    parser.add_argument('--objects',    type=str,  nargs='+',     help='objects')
    parser.add_argument('--n_trains',   type=int,  nargs='+',     help='n_trains')
    parser.add_argument('--n_iters',    type=int,  default=50,    help='n_iters')
    parser.add_argument('--skip',       type=bool, default=False, help='skip')
    parser.add_argument('--debug',      type=bool, default=False, help='visualization')
    args = parser.parse_args()

    if len(args.video) > 0 and not os.path.isdir(os.path.dirname(args.video)):
        print("[Error] Invalid path: ", args.video)
        exit(0)

    if args.method not in ['mdiff_cell',  'rand_cell', 'grand_cell', 'bf_cell', 'grid_cell', 'fdiff_cell', 'diff_cell',  'cma_cell',  'nm_cell',  'adiff_cell',  'asdiff_cell',
                           'mdiff_group', 'rand_group','grand_group','bf_group','grid_group','fdiff_group','diff_group', 'cma_group', 'nm_group', 'adiff_group', 'asdiff_group']:
        print('[Error] Invalid method: ', args.method)
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
        elif args.method.startswith('adiff'):
            engine = ActiveDiffPhysics(mode='multi')
        elif args.method.startswith('asdiff'):
            engine = ActiveDiffPhysics(mode='single')
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

    obj2ngroups = {'book':28,'box':65,'cramp':86,'crimp':44,'hammer':88,'ranch':52,'snack':64,'toothpaste':74,'windex':84}

    if args.param<=0:
        args.param = range(10)
    else:
        args.param = [args.param - 1]

    if args.sets <=0:
        if args.indir=='':
            args.sets = range(len(idxes_sims[0]))
        else:
            args.sets = range(len(idxes_real[0]))
    else:
        args.sets = [args.sets-1]

    if args.n_trains is None or len(args.n_trains)==0:
        args.n_trains = range(5,0,-1)

    # simulation
    if args.indir == '':
        if args.objects is None or len(args.objects)==0:
            #args.objects = ['book','box','cramp','crimp','hammer','ranch','snack','toothpaste','windex']
            args.objects = ['book','box','crimp','hammer','ranch','snack','toothpaste','windex']
        
        if args.method.endswith('group'):
            n_groups = [3,10,20]
        else:
            n_groups = [-1]

        #times_sims = []
        #times_grads = []

        for idx_param in args.param:
            for obj in args.objects:
                for n_trains in args.n_trains:
                    n_group = -1
                    for idx in args.sets:
                        print('param:%d obj:%s n_train:%d set:%d' % (idx_param,obj,n_trains,idx))
                        if args.skip:
                            fp_save = os.path.join(args.outdir,
                                                   '%s_%s_param%d_train%d_set%d_c%d.pkl'%\
                                                   (args.method,obj,idx_param,n_trains,idx,obj2ngroups[obj]))
                            if os.path.isfile(fp_save):
                                print('[SKIP] %s'%fp_save)
                                continue

                        idxes_train, idxes_test = idxes_sim[n_trains-1][idx]
                        engine = get_engine(args.method)
                                   
                        if args.method.endswith('_cell'):
                            if args.method.startswith('adiff') or args.method.startswith('asdiff'):
                                setups = get_setups_tools_sim([obj],None,None,idx_param)
                                engine.setup(setups)
                                history = engine.infer(n_trains, idxes_train[0])
                            else:
                                setups = get_setups_tools_sim([obj],idxes_train,None,idx_param)
                                engine.setup(setups, n_group=n_group)
                                history = engine.infer(infer_type='cell_mass',nsimslimit=500)

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
                                               '%s_%s_param%d_train%d_set%d_c%d.pkl'%\
                                               (args.method,obj,idx_param,n_trains,idx,n_group))

                        with open(fp_save,'wb') as f:
                            pickle.dump(summary, f, protocol=2)

                        #times_sims.append( history[-1]['time_sims']/float(history[-1]['n_sims']) )
                        #times_grads.append(history[-1]['time_grads']/float(history[-1]['n_grads']))
    
        #print('average time / a simulation: {} (\pm{})'.format(np.mean(times_sims),  np.std(times_sims)))
        #print('average time / a gradient: {} (\pm{})'.format(  np.mean(times_grads), np.std(times_grads)))

    # real experiment
    else:

        if args.objects is None or len(args.objects)==0:
            args.objects = ['book','box','hammer','snack','windex']

        for obj in args.objects:

            fp_label = os.path.join('../../dataset/objects',obj+'_label.pgm')
            setup = {'actor':{'name':'finger','type':'finger'},
                     'target_infos':[{'name':obj,'label':fp_label,'masses':[1,1,1],'meter_per_pixel':0.53/640.0}],
                     'mass_minmax':[0.1, 5.0],
                     'train':[],
                     'test':[],
                     'real':[]}

            files = glob.glob(os.path.join(args.indir, obj+'_*_traj.txt'))
            for idx_f in range(1,len(files)+1):
                file = os.path.join(args.indir, '%s_push%d_traj.txt' % (obj,idx_f))

                f = open(file,'r')
                node = yaml.load(f, Loader=yaml.BaseLoader)
                f.close()

                vec_x = float(node['push']['vec']['x'])
                vec_x = vec_x/abs(vec_x)

                push_x = float(node['push']['x']) - float(node['begin']['center_of_pixels']['x']) - vec_x*0.10
                push_y = float(node['push']['y']) - float(node['begin']['center_of_pixels']['y'])

                x_res = float(node['transform']['x'])
                y_res = float(node['transform']['y'])                
                yaw_res = float(node['transform']['yaw'])
                
                action = {'finger':{'pos': [push_x,push_y], 'yaw':0,
                                    'velocity':[float(node['push']['vec']['x']) + vec_x*0.10,
                                                float(node['push']['vec']['y']),
                                                float(node['push']['vec']['z'])],
                                    'duration':1.0},
                          'targets':[obj], 
                          obj:{'pos':[0,0], 'yaw':0, 
                               'pos_res':[x_res,y_res], 'yaw_res':yaw_res}
                         }
                setup['real'].append(action)

            for idx in args.sets:
                for n_trains in args.n_trains:
                    if args.skip:
                        fp_save = os.path.join(args.outdir,
                                               '%s_%s_real_train%d_set%d_c%d.pkl'%\
                                               (args.method,obj,n_trains,idx,obj2ngroups[obj]))
                        if os.path.isfile(fp_save):
                            print('[SKIP] %s'%fp_save)
                            continue

                    print('real obj:%s n_train:%d set:%d' % (obj,n_trains,idx))

                    idxes_train, idxes_test = idxes_real[n_trains-1][idx]
                    engine = get_engine(args.method)

                    if args.method.startswith('adiff') or args.method.startswith('asdiff'):
                        setup['train'] = range(7)
                        setup['test'] = range(7,14)
                        engine.setup(setup)
                        history = engine.infer(n_trains, idxes_train[0])
                    else:
                        setup['train'] = idxes_train
                        setup['test'] = range(7,14)
                        engine.setup(setup, n_group=-1)
                        history = engine.infer(infer_type='cell_mass',nsimslimit=500,lr_mass=0.5)
        
                    print('[Test Result] cell err: %.3f (m) pose err: %.3f (m) %.3f (deg)' % 
                           (history[-1]['test_error_cell'],
                            history[-1]['test_error_xy'],
                            history[-1]['test_error_yaw']) )
        
                    summary = {'object':obj,
                               'idxes_train':idxes_train,
                               'idxes_test':idxes_test,
                               'history':history,
                              }
        
                    n_group = len(engine.meta_targets[obj]['cells']['cells'])
                    fp_save = os.path.join(args.outdir,
                                           '%s_%s_real_train%d_set%d_c%d.pkl'%\
                                           (args.method,obj,n_trains,idx,n_group))
                    with open(fp_save,'wb') as f:
                        pickle.dump(summary, f, protocol=2)
