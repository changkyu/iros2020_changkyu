import os, sys
import pickle
import glob
import numpy as np

obj2ncell = {'hammer':73, 'crimp':94, 'ranch':107, 'cramp':110}

if __name__=='__main__':

    name = 'diff_cell'
    print('train\ttest\tname')
    #for obj in ['hammer', 'crimp', 'ranch', 'cramp']:
    for obj in ['cramp']:
        files = glob.glob('./res/%s_%s*_train5_*.pkl' % (name,obj))

        errs_train = []
        errs_test = []
        for file in files:
            f = open(file,'rb')
            res = pickle.load(f)
            f.close()

            _, filename = os.path.split(file)

            print('%.3f\t%.3f\t%s' %( res['history'][-1]['train_error_cell'],\
                                    res['history'][-1]['test_error_cell'], filename ))

            err_train = res['history'][-1]['train_error_cell']
            err_test = res['history'][-1]['test_error_cell']
            errs_train.append(err_train)
            errs_test.append(err_test)

        err_mean_train = np.mean(np.array(errs_train))
        err_mean_test  = np.mean(np.array(errs_test))
        err_std_train = np.std(np.array(errs_train))
        err_std_test  = np.std(np.array(errs_test))

        print('%.3f(+-%.3f)\t%.3f(+-%.3f)' % (err_mean_train,err_std_train,
                                             err_mean_test, err_std_test  ))
        print