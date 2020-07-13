import sys, os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
import progressbar
import yaml
import glob
import pickle

def load_resfile(fp, keys):

    f = open(fp,'rb')
    res = pickle.load(f)
    f.close()

    ret = {}
    for key in keys:
        ret[key] = res[key]
    return ret

def load_resdir(indir, names, keys):

    data = {}
    for name in names:
        res_all = {}
        dat = {}
        for obj in objects:
            res = {}
            dat[obj] = {}
            for idx_set in range(10):
                pattern = os.path.join(indir,name,'%s_%s_param*_initgoal%d.pkl' % (name,obj,idx_set))
                files = glob.glob(pattern)
                #print(pattern)
                print('method:%s obj:%s # of res:%d' % (name,obj,len(files)))
                for file in files:
                    ret = load_resfile(file, keys)
                    for key in keys:
                        if key not in res:
                            res[key] = []
                        res[key].append(ret[key])

            for key in keys:
                if key not in res_all:
                    res_all[key] = []
                res_all[key] = res_all[key] + res[key]
            dat[obj]['aver'] = {}
            dat[obj]['std'] = {}
            dat[obj]['aver'][key] = np.mean(res[key])
            dat[obj]['std'][key]  =  np.std(res[key])
        dat['aver'] = {}
        dat['std'] = {}
        dat['aver'][key] = np.mean(res_all[key])
        dat['std'][key]  =  np.std(res_all[key])
        data[name] = dat
    return data


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

method2name = { 'diff'      :'Proposed Algorithm',
                'bfs'       :'Exhaustive Search',
                'diffclosed':'Identified Mass Distribution with Friction Upper Bound',
                'fricclosed':'Identified Mass Distribution without Friction Upper Bound',
                'noneclosed':'Uniform Mass Distribution',
              }

labels = { 'time' : 'time (sec)',
           'sims' : 'number of simulations',
           'cell' : 'mean error of cell positions (m)',
           'xy'   : 'location error',
           'yaw'  : 'angular error',                   }

palette = {'bfs'            :'royalblue',
           'noneclosed'     :'royalblue',
           'bfs_std'        :'navy',
           'noneclosed_std' :'navy',           
           'diff'           :'crimson',
           'diffclosed'     :'crimson',
           'diff_std'       :'maroon',
           'diffclosed_std' :'maroon',
           'fricclosed'     :'green'
           }

objects = ['book','box','crimp','hammer','ranch','snack','toothpaste','windex']

if __name__=='__main__':

    if not os.path.isdir('fig'):
        os.makedirs('fig')

    methods = ['bfs','diff']
    names_methods = [method2name[method] for method in methods]
    names_objects = [object2name[obj] for obj in objects]
    dir_planonly_res = '/home/cs1080/projects/rss2019/results/plan/planonly'
    data = load_resdir(dir_planonly_res, methods, ['n_sims'])

    fig = plt.figure()
    ax = fig.add_subplot(111)
    width = 0.4
    for m, method in enumerate(methods):
        vals = []
        stds = []
        for o, obj in enumerate(objects):
            val = data[method][obj]['aver']['n_sims']
            std = data[method][obj]['std']['n_sims']
            print('%s %s %.3f (\pm%.3f)' % (obj,method,val,std))
            vals.append(val)
            stds.append(std)
        ax.bar( [idx+m*width for idx in range(len(vals))], vals, 
                 width=width, yerr=stds, capsize=5,
                 color=palette[method], ecolor=palette[method+'_std'])
    ax.set(ylabel=labels['sims'])
    ax.set_xticks([idx+m*width*0.5 for idx in range(len(vals))])
    ax.set_xticklabels(names_objects, rotation=15)
    ax.set_title('Number of Simulations Used for a Planning')
    ax.legend(names_methods)
    fig.tight_layout()
    fp_savefig = os.path.join('./fig/','plan_nsims.png')
    fig.savefig(fp_savefig, dpi=600)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    methods = ['noneclosed','fricclosed','diffclosed']
    names_methods = [method2name[method] for method in methods]
    data_real = {}
    data_real['diffclosed'] = 16/16.0
    data_real['noneclosed'] = 12/16.0
    data_real['fricclosed'] = 13/16.0

    #width = 0.2
    #for m, method in enumerate(methods):
    #    ax.barh( m*width, data_real[method]*100, 
    #            height=width, color=palette[method] )
    #ax.set(xlabel='success rate (%) for grasping')
    #ax.set_xlim([50, 100])
    #ax.set_ylim([width*3,-width*2])
    #ax.set_yticks([])

    ax.grid(zorder=0)
    for m, method in enumerate(methods):
        ax.bar( m*1.5, data_real[method]*100, color=palette[method], zorder=3 )
    ax.set(ylabel='success rate (%) for grasping')    
    ax.set_xticks([])
    ax.set_yticks([50,60,70,80,90,100])
    bottom, top = ax.get_ylim()
    ax.set_xlim([-1, len(names_methods)*1.5 - 0.5])
    ax.set_ylim([50, 115])
    ax.set_title('Succes Rate for Grasping')
    
    ax.legend(names_methods, loc='upper right')
    fig.tight_layout()
    fp_savefig = os.path.join('./fig/','plan_grasp.png')
    fig.savefig(fp_savefig, dpi=600)
    plt.show()