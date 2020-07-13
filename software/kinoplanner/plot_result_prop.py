import os, sys
import pickle
import glob
import numpy as np

import seaborn as sns
import matplotlib
import matplotlib.pyplot as plt

def load_resdir(dir_res, method, obj):

    succs = []
    plan_lengths = []

    if method.startswith('det'):
        dir_sav = '/home/cs1080/projects/iros2020/software/cellphysics/sav/'

        probs = []
        if method=='det_sample':
            for i in range(1,10):
                with open(dir_sav+'/'+obj+'_seed'+str(i)+'.pkl','rb') as f:
                    tmp = pickle.load(f)
                    probs.append(np.exp(-tmp['error_cell']))
        else:
            probs = [1] * 9
        probs_sum = sum(probs)
        probs = [prob/float(probs_sum) for prob in probs]

        for i in range(1,10):
            files = glob.glob(os.path.join(dir_res,'%s%d_%s_init*.pkl' %('det',i,obj)))
            for file in files:
                with open(file,'rb') as f:
                    res = pickle.load(f)
                    succs.append(res['success']*probs[i-1]*9)
                    if succs[-1]:
                        plan_lengths.append(res['plan_length']*probs[i-1]*9)

    else:
        files = glob.glob(os.path.join(dir_res,'%s_%s_init*.pkl' %(method,obj)))
        for file in files:
            with open(file,'rb') as f:
                res = pickle.load(f)
                succs.append(res['success'])
                if succs[-1]:
                    plan_lengths.append(res['plan_length'])

    return {'success_rate'     : np.mean(succs)*100,
            'plan_length_mean' : np.mean(plan_lengths),
            'plan_length_std'  : np.std(plan_lengths)
           }

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

method2name = {'none'   :'Planning without Mass Identification',
               'uniform':'Planning with Uniform Mass',
               'prop'   :'Proposed Algorithm',
               'known'  :'Planning with Ground Truth Mass',
               'det_aver' :'Planning with Deterministic Mass Identification',
               'det_sample':'Planning with Sampling on Deterministic Mass Identifications'
              }

palette = {'none':'navy',
           'none_std':'black',
           'uniform':'navy',
           'uniform_std':'black',
           'det':'green',
           'det_std':'darkgreen',
           'det_aver':'green',
           'det_aver_std':'darkgreen',
           'det_sample':'green',
           'det_sample_std':'darkgreen',           
           'known':'orange',
           'known_std':'darkorange',
           'prop':'crimson',
           'prop_std':'maroon',
           }

if __name__=='__main__':

    if not os.path.isdir('fig'):
        os.makedirs('fig')

    methods = ['uniform','det_sample','known','prop']
    #methods = ['none', 'uniform']
    #objects = ['book','box','crimp','hammer','ranch','snack','toothpaste','windex']
    objects = ['book','box','hammer','ranch','snack','windex']
    dir_res = 'res_prop'

    succ = {}
    plan_mean = {}
    plan_std = {}
    for obj in objects:
        succ[obj] = {}
        plan_mean[obj] = {}    
        plan_std[obj] = {}    
        for method in methods:
            res = load_resdir(dir_res, method, obj)
            succ[obj][method]      = res['success_rate']
            plan_mean[obj][method] = res['plan_length_mean']
            plan_std[obj][method]  = res['plan_length_std']

    fig = plt.figure()
    ax = fig.add_subplot(111)
    names_methods = [method2name[method] for method in methods]
    names_objects = [object2name[obj] for obj in objects]
    width = 1/float(len(methods)+2)
    for m, method in enumerate(methods):
        vals = []
        stds = []
        for o, obj in enumerate(objects):
            val = plan_mean[obj][method]
            std = plan_std[obj][method]
            vals.append(val)
            stds.append(std)        
        ax.bar( [idx+m*width for idx in range(len(vals))], 
                vals, width=width, color=palette[method],
                yerr=stds, capsize=4, ecolor=palette[method+'_std']
               )
    ax.set(ylabel='Number of Executed Actions')
    ax.set_xticks([idx+m*width*0.5 for idx in range(len(vals))])
    ax.set_xticklabels(names_objects, rotation=15)
    #ax.set_yticks(range(0,120,20))    
    ax.set_ylim([0, 16])
    ax.set_title('Length of the Planned Trajectory')
    ax.legend(names_methods)
    fig.tight_layout()
    fp_savefig = os.path.join('./fig/','plan_length.png')
    fig.savefig(fp_savefig, dpi=600)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.grid(zorder=0)
    for m, method in enumerate(methods):
        vals = []
        for o, obj in enumerate(objects):
            val = succ[obj][method]
            vals.append(val)
        ax.bar( [idx+m*width for idx in range(len(vals))], 
                vals, width=width, color=palette[method],
                zorder=3
               )
    ax.set(ylabel='Success Rate (%)')
    ax.set_xticks([idx+m*width*0.5 for idx in range(len(vals))])
    ax.set_xticklabels(names_objects, rotation=15)
    ax.set_yticks(range(0,120,20))    
    ax.set_ylim([0, 140])
    ax.set_title('Success Rate')
    ax.legend(names_methods)
    fig.tight_layout()
    fp_savefig = os.path.join('./fig/','plan_succs.png')
    fig.savefig(fp_savefig, dpi=600)

    #plt.show()