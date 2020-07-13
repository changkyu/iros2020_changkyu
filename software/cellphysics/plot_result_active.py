import os, sys
import pickle
import glob
import numpy as np

import seaborn as sns
import matplotlib
import matplotlib.pyplot as plt
import progressbar

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

method2name = { 'diff_cell': 'Random',
               'adiff_cell': 'Select the Most Variant Action among Multiple Models',
               'asdiff_cell':'Select the Most Different Action than Current Actions',
              }

palette = {'diff_cell'     :'navy',
           'diff_cell_std' :'navy',
           'adiff_cell'    :'crimson',
           'adiff_cell_std':'crimson',
           'asdiff_cell'    :'orange',
           'asdiff_cell_std':'orange',
          }

prefix = 'sims'
#prefix = 'real'

if prefix=='sims':
    objects = ['book','box','crimp','hammer','ranch','snack','toothpaste','windex']
else:
    objects = ['book','box','hammer','snack','windex']

methods = ['diff_cell','asdiff_cell','adiff_cell']

n_trains = range(1,6)

def load_resfile(file, n_train):

    f = open(file,'rb')
    res = pickle.load(f)
    f.close()

    err_xy    = [h['test_error_xy']    for h in res['history']]
    err_yaw   = [h['test_error_yaw']   for h in res['history']]
    err_cell  = [h['test_error_cell']  for h in res['history']]
    errs_xy   = [h['test_errors_xy']   for h in res['history']]
    errs_yaw  = [h['test_errors_yaw']  for h in res['history']]
    errs_cell = [h['test_errors_cell'] for h in res['history']]

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

    return {#'n_sims'       :n_sims,
            #'times'        :times,
            'sims2err_xy'  :sims2err_xy,
            'time2err_xy'  :time2err_xy,
            'sims2err_yaw' :sims2err_yaw,
            'time2err_yaw' :time2err_yaw,
            'sims2err_cell':sims2err_cell,
            'time2err_cell':time2err_cell,
            'sims2std_xy'  :sims2std_xy,
            'time2std_xy'  :time2std_xy,
            'sims2std_yaw' :sims2std_yaw,
            'time2std_yaw' :time2std_yaw,
            'sims2std_cell':sims2std_cell,
            'time2std_cell':time2std_cell,
           }

def load_resdir(dir_res):

    data = {}
    for obj in objects:
        data[obj] = {}
        for method in methods:
            data[obj][method] = {'aver':[], 'std':[], 'sims2err_cell':[]}
            for n_train in n_trains:
                res_all = {}
                pattern = os.path.join(dir_res,'%s_%s_param*_train%d_set*.pkl' % (method, obj, n_train))    
                files = glob.glob(pattern)
                for file in files:
                    print(file)
                    res = load_resfile(file, n_train)
                    for key in res:
                        if key not in res_all:
                            res_all[key] = [res[key]]
                        else:
                            res_all[key].append(res[key])

                res_aver = {}
                for key in res_all:
                    res_aver[key] = {}
                    res_aver[key]['aver'] = np.mean(res_all[key], axis=0)
                    res_aver[key]['std']  = np.std( res_all[key], axis=0)

                data[obj][method]['sims2err_cell'].append(
                    {'aver':res_aver['sims2err_cell']['aver'], 
                      'std':res_aver['sims2err_cell']['std']})
                data[obj][method]['aver'].append(res_aver['sims2err_cell']['aver'][5])
                data[obj][method]['std'].append(res_aver['sims2err_cell']['std'][5])
    return data

if __name__=='__main__':

    dir_res = './res'
    #objects = ['book']
    data = load_resdir(dir_res)

    for obj in objects:
        for t, n_train in enumerate(n_trains):
            fig = plt.figure()
            ax = fig.add_subplot(111)

            for method in methods:
                nsims = range(1,data[obj][method]['sims2err_cell'][t]['aver'].shape[0]+1)
                ax.plot(nsims,data[obj][method]['sims2err_cell'][t]['aver'], color=palette[method])
                ax.fill_between(nsims,
                                data[obj][method]['sims2err_cell'][t]['aver']-data[obj][method]['sims2err_cell'][t]['std']*0.5,
                                data[obj][method]['sims2err_cell'][t]['aver']+data[obj][method]['sims2err_cell'][t]['std']*0.5,
                                color=palette[method+'_std'], alpha=0.1)
            
            ax.set_title('n_train:%d' % n_train)
            ax.set_xticks(nsims)
            ax.set(xlabel='Number of Simulation', ylabel='Average Error of Predicted Cell Positions (m)')
            ax.legend([method2name[m] for m in methods],loc='upper right')
            fig.tight_layout()
            fp_savefig = os.path.join('./fig/','%s_%s_ntrain%d.png' % (prefix,obj,n_train))
            fig.savefig(fp_savefig, dpi=600)
            plt.close(fig)

    for obj in objects:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for method in methods:
            ax.plot(n_trains,data[obj][method]['aver'], color=palette[method])
            ax.fill_between(n_trains,
                            np.array(data[obj][method]['aver'])-np.array(data[obj][method]['std'])*0.5,
                            np.array(data[obj][method]['aver'])+np.array(data[obj][method]['std'])*0.5,
                            color=palette[method+'_std'], alpha=0.1)
        ax.set_title(obj)
        ax.set_xticks(n_trains)
        ax.set(xlabel='Number of Traning Data', ylabel='Average Error of Predicted Cell Positions (m)')
        ax.legend([method2name[m] for m in methods],loc='upper right')
        fig.tight_layout()
        fp_savefig = os.path.join('./fig/','%s_%s.png' % (prefix,obj))
        fig.savefig(fp_savefig, dpi=600)
        plt.close(fig)
    