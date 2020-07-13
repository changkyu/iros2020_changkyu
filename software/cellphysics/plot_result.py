import sys, os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
import progressbar
import pandas
import yaml
import glob
import pickle
import itertools

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

def load_resfile(fp, n_train, vis=False):

    f = open(fp,'rb')
    res = pickle.load(f)
    f.close()

    directory, filename = os.path.split(fp)
    if False:#filename.startswith('mdiff'):
        err_xy   = [h['test_error_xy_lcp']   for h in res['history']]
        err_yaw  = [h['test_error_yaw_lcp']  for h in res['history']]
        err_cell = [h['test_error_cell_lcp'] for h in res['history']]
        errs_xy   = [h['test_errors_xy_lcp']   for h in res['history']]
        errs_yaw  = [h['test_errors_yaw_lcp']  for h in res['history']]
        errs_cell = [h['test_errors_cell_lcp'] for h in res['history']]
    else:
        err_xy   = [h['test_error_xy']      for h in res['history']]
        err_yaw  = [h['test_error_yaw']     for h in res['history']]
        err_cell = [h['test_error_cell']    for h in res['history']]
        errs_xy   = [h['test_errors_xy']      for h in res['history']]
        errs_yaw  = [h['test_errors_yaw']     for h in res['history']]
        errs_cell = [h['test_errors_cell']    for h in res['history']]

    n_sims   = [h['n_sims']/n_train for h in res['history']]
    times    = [int(np.ceil(h['time'])) for h in res['history']]

    sims2err_xy   = np.zeros([50/n_train,])
    time2err_xy   = np.zeros([50/n_train,])
    sims2err_yaw  = np.zeros([50/n_train,])
    time2err_yaw  = np.zeros([50/n_train,])
    sims2err_cell = np.zeros([50/n_train,])
    time2err_cell = np.zeros([50/n_train,])

    sims2std_xy   = np.zeros([50/n_train,])
    time2std_xy   = np.zeros([50/n_train,])
    sims2std_yaw  = np.zeros([50/n_train,])
    time2std_yaw  = np.zeros([50/n_train,])
    sims2std_cell = np.zeros([50/n_train,])
    time2std_cell = np.zeros([50/n_train,])
    
    sims2err_xy[  :] =   err_xy[0]
    time2err_xy[  :] =   err_xy[0]
    sims2err_yaw[ :] =  err_yaw[0]
    time2err_yaw[ :] =  err_yaw[0]
    sims2err_cell[:] = err_cell[0]
    time2err_cell[:] = err_cell[0]

    sims2std_xy[  :] = np.std(np.array(  errs_xy[0]))
    time2std_xy[  :] = np.std(np.array(  errs_xy[0]))
    sims2std_yaw[ :] = np.std(np.array( errs_yaw[0]))
    time2std_yaw[ :] = np.std(np.array( errs_yaw[0]))
    sims2std_cell[:] = np.std(np.array(errs_cell[0]))
    time2std_cell[:] = np.std(np.array(errs_cell[0]))

    for i in range(0,len(n_sims)):

        if n_sims[i] >= len(sims2err_cell):
            break

        sims2err_xy[  n_sims[i]-1:] =   err_xy[i]
        time2err_xy[   times[i]-1:] =   err_xy[i]
        sims2err_yaw[ n_sims[i]-1:] =  err_yaw[i]
        time2err_yaw[  times[i]-1:] =  err_yaw[i]
        sims2err_cell[n_sims[i]-1:] = err_cell[i]
        time2err_cell[ times[i]-1:] = err_cell[i]

        sims2std_xy[  n_sims[i]-1:] = np.std(np.array(  errs_xy[i]))
        time2std_xy[   times[i]-1:] = np.std(np.array(  errs_xy[i]))
        sims2std_yaw[ n_sims[i]-1:] = np.std(np.array( errs_yaw[i]))
        time2std_yaw[  times[i]-1:] = np.std(np.array( errs_yaw[i]))
        sims2std_cell[n_sims[i]-1:] = np.std(np.array(errs_cell[i]))
        time2std_cell[ times[i]-1:] = np.std(np.array(errs_cell[i]))

    #print(n_sims)
    #print(times)
    #print(err_cell)

    if vis:
        plt.subplot(221)
        plt.plot(n_sims, err_cell)
        plt.subplot(222)
        plt.plot(range(1,sims2err_cell.shape[0]+1), sims2err_cell)
        plt.subplot(223)
        plt.plot(times, err_cell)
        plt.subplot(224)
        plt.plot(range(1,time2err_cell.shape[0]+1), time2err_cell)    
        plt.show()

    return (n_sims,\
        times,\
        sims2err_xy,\
        time2err_xy,\
        sims2err_yaw,\
        time2err_yaw,\
        sims2err_cell,\
        time2err_cell,\
        sims2std_xy,\
        time2std_xy,\
        sims2std_yaw,\
        time2std_yaw,\
        sims2std_cell,\
        time2std_cell,\
        )


def load_resdir(indir, names, n_train, n_cell=-1):

    for name in names:
        dat = {}
        sims2errs_all_xy = []
        time2errs_all_xy = []
        sims2errs_all_yaw = []
        time2errs_all_yaw = []
        sims2errs_all_cell = []
        time2errs_all_cell = []        
        for obj in objects:

            sims2errs_aver_xy = []
            time2errs_aver_xy = []
            sims2errs_aver_yaw = []
            time2errs_aver_yaw = []
            sims2errs_aver_cell = []
            time2errs_aver_cell = []
            sims2stds_aver_xy = []
            time2stds_aver_xy = []
            sims2stds_aver_yaw = []
            time2stds_aver_yaw = []
            sims2stds_aver_cell = []
            time2stds_aver_cell = []

            dat[obj] = {}
            for idx_set in obj2sets[prefix][obj]:
                dat[obj][idx_set] = {}

                sims2errs_xy = []
                time2errs_xy = []
                sims2errs_yaw = []
                time2errs_yaw = []
                sims2errs_cell = []
                time2errs_cell = []
                sims2stds_xy = []
                time2stds_xy = []
                sims2stds_yaw = []
                time2stds_yaw = []
                sims2stds_cell = []
                time2stds_cell = []

                if prefix=='sims':
                    pattern = os.path.join(indir,name.split('_')[0],'%s_%s_param*_train%d_set%d_c%d.pkl' % (name,obj,n_train,idx_set,obj2ncell[obj]))
                    files = glob.glob(pattern)
                    print(pattern)
                    print('method:%s obj:%s # of res:%d' % (name,obj,len(files)))
                    for file in files:
                        (n_sims,\
                         times,\
                         sims2err_xy,\
                         time2err_xy,\
                         sims2err_yaw,\
                         time2err_yaw,\
                         sims2err_cell,\
                         time2err_cell,\
                         sims2std_xy,\
                         time2std_xy,\
                         sims2std_yaw,\
                         time2std_yaw,\
                         sims2std_cell,\
                         time2std_cell) = load_resfile(file, n_train)

                        sims2errs_xy.append(sims2err_xy)
                        time2errs_xy.append(time2err_xy)
                        sims2errs_yaw.append(sims2err_yaw)
                        time2errs_yaw.append(time2err_yaw)
                        sims2errs_cell.append(sims2err_cell)
                        time2errs_cell.append(time2err_cell)
                        sims2stds_xy.append(sims2std_xy)
                        time2stds_xy.append(time2std_xy)
                        sims2stds_yaw.append(sims2std_yaw)
                        time2stds_yaw.append(time2std_yaw)
                        sims2stds_cell.append(sims2std_cell)
                        time2stds_cell.append(time2std_cell)
                else:
                    pattern = os.path.join(indir,name.split('_')[0],'%s_%s_real_set%d_c%d.pkl' % (name,obj,idx_set,obj2ncell[obj]))
                    files = glob.glob(pattern)
                    print(pattern)
                    print('method:%s obj:%s # of res:%d' % (name,obj,len(files)))
                    for file in files:
                        (n_sims,\
                         times,\
                         sims2err_xy,\
                         time2err_xy,\
                         sims2err_yaw,\
                         time2err_yaw,\
                         sims2err_cell,\
                         time2err_cell,\
                         sims2std_xy,\
                         time2std_xy,\
                         sims2std_yaw,\
                         time2std_yaw,\
                         sims2std_cell,\
                         time2std_cell) = load_resfile(file, n_train)

                        sims2errs_xy.append(sims2err_xy)
                        time2errs_xy.append(time2err_xy)
                        sims2errs_yaw.append(sims2err_yaw)
                        time2errs_yaw.append(time2err_yaw)
                        sims2errs_cell.append(sims2err_cell)
                        time2errs_cell.append(time2err_cell)
                        sims2stds_xy.append(sims2std_xy)
                        time2stds_xy.append(time2std_xy)
                        sims2stds_yaw.append(sims2std_yaw)
                        time2stds_yaw.append(time2std_yaw)
                        sims2stds_cell.append(sims2std_cell)
                        time2stds_cell.append(time2std_cell)

                sims2errs_all_xy   = sims2errs_all_xy   + sims2errs_xy
                time2errs_all_xy   = time2errs_all_xy   + time2errs_xy
                sims2errs_all_yaw  = sims2errs_all_yaw  + sims2errs_yaw
                time2errs_all_yaw  = time2errs_all_yaw  + time2errs_yaw
                sims2errs_all_cell = sims2errs_all_cell + sims2errs_cell
                time2errs_all_cell = time2errs_all_cell + time2errs_cell

                sims2errs_aver_xy   = sims2errs_aver_xy   + sims2errs_xy
                time2errs_aver_xy   = time2errs_aver_xy   + time2errs_xy
                sims2errs_aver_yaw  = sims2errs_aver_yaw  + sims2errs_yaw
                time2errs_aver_yaw  = time2errs_aver_yaw  + time2errs_yaw
                sims2errs_aver_cell = sims2errs_aver_cell + sims2errs_cell
                time2errs_aver_cell = time2errs_aver_cell + time2errs_cell
                sims2stds_aver_xy   = sims2stds_aver_xy   + sims2stds_xy
                time2stds_aver_xy   = time2stds_aver_xy   + time2stds_xy
                sims2stds_aver_yaw  = sims2stds_aver_yaw  + sims2stds_yaw
                time2stds_aver_yaw  = time2stds_aver_yaw  + time2stds_yaw
                sims2stds_aver_cell = sims2stds_aver_cell + sims2stds_cell
                time2stds_aver_cell = time2stds_aver_cell + time2stds_cell

                sims2errs_xy   = np.array(sims2errs_xy)
                time2errs_xy   = np.array(time2errs_xy)
                sims2errs_yaw  = np.array(sims2errs_yaw)
                time2errs_yaw  = np.array(time2errs_yaw)
                sims2errs_cell = np.array(sims2errs_cell)
                time2errs_cell = np.array(time2errs_cell)
                sims2stds_xy   = np.array(sims2stds_xy)
                time2stds_xy   = np.array(time2stds_xy)
                sims2stds_yaw  = np.array(sims2stds_yaw)
                time2stds_yaw  = np.array(time2stds_yaw)
                sims2stds_cell = np.array(sims2stds_cell)
                time2stds_cell = np.array(time2stds_cell)

                dat[obj][idx_set]['sims2err_xy']   = np.mean(sims2errs_xy,   axis=0)
                dat[obj][idx_set]['time2err_xy']   = np.mean(time2errs_xy,   axis=0)
                dat[obj][idx_set]['sims2err_yaw']  = np.mean(sims2errs_yaw,  axis=0)
                dat[obj][idx_set]['time2err_yaw']  = np.mean(time2errs_yaw,  axis=0)
                dat[obj][idx_set]['sims2err_cell'] = np.mean(sims2errs_cell, axis=0)
                dat[obj][idx_set]['time2err_cell'] = np.mean(time2errs_cell, axis=0)
                dat[obj][idx_set]['sims2std_xy']   = np.mean(sims2stds_xy,   axis=0)
                dat[obj][idx_set]['time2std_xy']   = np.mean(time2stds_xy,   axis=0)
                dat[obj][idx_set]['sims2std_yaw']  = np.mean(sims2stds_yaw,  axis=0)
                dat[obj][idx_set]['time2std_yaw']  = np.mean(time2stds_yaw,  axis=0)
                dat[obj][idx_set]['sims2std_cell'] = np.mean(sims2stds_cell, axis=0)
                dat[obj][idx_set]['time2std_cell'] = np.mean(time2stds_cell, axis=0)

            sims2errs_aver_xy   = np.array(sims2errs_aver_xy)
            time2errs_aver_xy   = np.array(time2errs_aver_xy)
            sims2errs_aver_yaw  = np.array(sims2errs_aver_yaw)
            time2errs_aver_yaw  = np.array(time2errs_aver_yaw)
            sims2errs_aver_cell = np.array(sims2errs_aver_cell)
            time2errs_aver_cell = np.array(time2errs_aver_cell)
            sims2stds_aver_xy   = np.array(sims2stds_aver_xy)
            time2stds_aver_xy   = np.array(time2stds_aver_xy)
            sims2stds_aver_yaw  = np.array(sims2stds_aver_yaw)
            time2stds_aver_yaw  = np.array(time2stds_aver_yaw)
            sims2stds_aver_cell = np.array(sims2stds_aver_cell)
            time2stds_aver_cell = np.array(time2stds_aver_cell)
            dat[obj]['aver'] = {}
            dat[obj]['aver']['sims2err_xy']   = np.mean(sims2errs_aver_xy,   axis=0)
            dat[obj]['aver']['time2err_xy']   = np.mean(time2errs_aver_xy,   axis=0)
            dat[obj]['aver']['sims2err_yaw']  = np.mean(sims2errs_aver_yaw,  axis=0)
            dat[obj]['aver']['time2err_yaw']  = np.mean(time2errs_aver_yaw,  axis=0)
            dat[obj]['aver']['sims2err_cell'] = np.mean(sims2errs_aver_cell, axis=0)
            dat[obj]['aver']['time2err_cell'] = np.mean(time2errs_aver_cell, axis=0)
            dat[obj]['aver']['sims2std_xy']   = np.std( sims2errs_aver_xy,    axis=0)
            dat[obj]['aver']['time2std_xy']   = np.std( time2errs_aver_xy,    axis=0)
            dat[obj]['aver']['sims2std_yaw']  = np.std( sims2errs_aver_yaw,   axis=0)
            dat[obj]['aver']['time2std_yaw']  = np.std( time2errs_aver_yaw,   axis=0)
            dat[obj]['aver']['sims2std_cell'] = np.std( sims2errs_aver_cell,  axis=0)
            dat[obj]['aver']['time2std_cell'] = np.std( time2errs_aver_cell,  axis=0)

        dat['aver'] = {'aver':{}}
        dat['aver']['aver']['sims2err_xy']   = np.mean(sims2errs_all_xy,   axis=0)
        dat['aver']['aver']['time2err_xy']   = np.mean(time2errs_all_xy,   axis=0)
        dat['aver']['aver']['sims2err_yaw']  = np.mean(sims2errs_all_yaw,  axis=0)
        dat['aver']['aver']['time2err_yaw']  = np.mean(time2errs_all_yaw,  axis=0)
        dat['aver']['aver']['sims2err_cell'] = np.mean(sims2errs_all_cell, axis=0)
        dat['aver']['aver']['time2err_cell'] = np.mean(time2errs_all_cell, axis=0)
        dat['aver']['aver']['sims2std_xy']   = np.std(sims2errs_all_xy,   axis=0)
        dat['aver']['aver']['time2std_xy']   = np.std(time2errs_all_xy,   axis=0)
        dat['aver']['aver']['sims2std_yaw']  = np.std(sims2errs_all_yaw,  axis=0)
        dat['aver']['aver']['time2std_yaw']  = np.std(time2errs_all_yaw,  axis=0)
        dat['aver']['aver']['sims2std_cell'] = np.std(sims2errs_all_cell, axis=0)
        dat['aver']['aver']['time2std_cell'] = np.std(time2errs_all_cell, axis=0)        

        data[name] = dat
    return data

data = {}

object2name = { 'book'  :'Book',
                'box'   :'Box',
                'cramp' :'Cramp',
                'crimp' :'Crimp',
                'hammer':'Hammer',
                'ranch' :'Ranch',
                'snack' :'Snack',
                'toothpaste':'Toothpaste',
                'windex':'Spray Gun',
                'aver'  :'Average',
              }

method2name = { 'diff_cell'  :'Proposed Algorithm',
                'diff_group' :'Approx. Diff. Physics (group)',
                'fdiff_cell' :'Finite Diff. Gradient',
                'fdiff_group':'Finite Diff. Gradient (group)',
                'mdiff_cell' :'Diff. Physics w/ Autograd',
                'mdiff_group':'Diff. Physics (group)',
                'bf_cell'    :'Brute Force',
                'bf_group'   :'Brute Force (group)',
                'grid_cell'  :'Grid Search',
                'grid_group' :'Grid Search (group)',
                'rand_cell'    :'Random Search',
                'rand_group'   :'Random Search (group)',
                'grand_cell'    :'Weighted Sampling',
                'grand_group'   :'Weighted Sampling (group)',
                'nm_cell'     :'Nelder-Mead',
                'cma_cell'    :'CMA-ES'
              }

labels = { 'time' : 'time (sec)',
           'sims' : 'number of simulations',
           'cell' : 'average error of predicted cell positions (m)',
           'xy'   : 'location error',
           'yaw'  : 'angular error',                   }

palette = {'rand_cell'      :'green',
           'grand_cell'     :'royalblue',
           'fdiff_cell'     :'navy',
           'nm_cell'        :'purple',
           'cma_cell'       :'orange',
           'diff_cell'      :'red',
           }

methods_group = [ 'bf_group',
                  'grid_group',
                  'fdiff_group',
                  'diff_group', ]

methods_cell = [ #'bf_cell',
                 #'grid_cell',
                 'rand_cell',
                 'grand_cell',
                 'fdiff_cell',
                 'cma_cell',
                 'nm_cell',
                 #'mdiff_cell',                 
                 'diff_cell',  ]

#methods_cell = ['fdiff_cell']

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

    if not os.path.isdir('fig'):
        os.makedirs('fig')

    dir_res = '/home/cs1080/projects/rss2019/results/prop/'+prefix

    n_train = 5
    data = load_resdir(dir_res, methods_cell, n_train)

    name_x = 'time'
    name_x = 'sims'
    name_y = 'cell'
    #name_y = 'yaw'
    name_err = name_x + '2err_' + name_y
    name_std = name_x + '2std_' + name_y

    matplotlib.rc('axes', labelsize=12) 

    for obj in objects + ['aver']:
        sys.stdout.write(' & %s' % obj)
    sys.stdout.write('\n')

    for method in methods_cell:
        sys.stdout.write('%s & ' % method2name[method])
        for obj in objects:
            sys.stdout.write('%s: '%obj)
            data_sorted = sorted(data[method][obj].items(), key=lambda x: x[1][name_err][-1])
            for idx_set, d in data_sorted:
                if type(idx_set)==str:
                    continue
                sys.stdout.write('[%d] %.3f & ' % (idx_set, d[name_err][-1]))
            #for idx_set in range(10):
            #    sys.stdout.write('[%d] %.3f (\pm%.3f) & '\
            #                       % (idx_set,\
            #                          data[method][obj][idx_set][name_err][-1],\
            #                          data[method][obj][idx_set][name_std][-1] ))
            sys.stdout.write('\n')
        sys.stdout.write('\n')

    if True:
        for obj in objects + ['aver']:
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.set_title(object2name[obj])
            y_max = 0
            for method in methods_cell:
                if y_max < np.max(data[method][obj]['aver'][name_err]):
                    y_max = np.max(data[method][obj]['aver'][name_err])

                ax.plot([n*n_train for n in range(1,len(data[method][obj]['aver'][name_err])+1)],data[method][obj]['aver'][name_err],color=palette[method])
                if method.startswith('diff'):
                    ax.fill_between([n*n_train for n in range(1,len(data[method][obj]['aver'][name_err])+1)],
                      data[method][obj]['aver'][name_err]-data[method][obj]['aver'][name_std],
                      data[method][obj]['aver'][name_err]+data[method][obj]['aver'][name_std],
                      color=palette[method], alpha=0.1
                    )
            ax.legend([method2name[m] for m in methods_cell],loc='upper right')

            print('{} {}'.format(obj,data[method][obj]['aver'][name_err][-1]))

            if y_max < 0.01:
                ax.set_ylim([0, 0.01])
            else:
                bottom, top = ax.get_ylim()

                if prefix=='sims':
                    if obj in ['hammer', 'toothpaste']:
                        gap = 0.035                    
                    elif obj in ['book','windex','box']:
                        gap = 0.025
                    elif obj in ['crimp']:
                        gap = 0.018
                    else:
                        gap = 0.02
                else:
                    if obj in ['box']:
                        gap = 0.010
                    elif obj in ['snack']:
                        gap = 0.025
                    elif obj in ['windex']:
                        gap = 0.0175
                    else:
                        gap = 0.02

                ax.set_ylim(bottom,y_max+gap)

            ax.set(xlabel=labels[name_x], ylabel=labels[name_y])
            fig.tight_layout()
            fp_savefig = os.path.join('./fig/',prefix+'_'+obj+'_'+name_x+'_vs_'+name_y+'.png')
            fig.savefig(fp_savefig, dpi=600)

    if True:
        obj = 'aver'
        method = 'diff_cell'
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title(object2name[obj])
        strs_ntrain = []
        for n_train in range(1,6):
            strs_ntrain.append('# of training examples = %d' % n_train)
            data = load_resdir(dir_res, [method], n_train)
            ax.plot([n*n_train for n in range(1,11)],data[method][obj]['aver'][name_err][:10])
        if prefix=='real':
            ax.legend(strs_ntrain, loc='upper right')
        else:
            ax.legend(strs_ntrain, loc='upper right')
            bottom, top = ax.get_ylim()
            ax.set_ylim(bottom,y_max+0.03)

        ax.set(xlabel='number of simulations per an training example', ylabel=labels[name_y])
        #ax.set_xscale('log')
        ax.set_yscale('log')
        fig.tight_layout()
        fp_savefig = os.path.join('./fig/',prefix+'_ntrain_'+name_x+'_vs_'+name_y+'.png')
        fig.savefig(fp_savefig, dpi=600)
    #plt.show()