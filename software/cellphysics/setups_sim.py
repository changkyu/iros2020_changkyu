import cv2
import matplotlib.pyplot as plt

def gen_actions_sim(img):

    if type(img)==str:
        img = cv2.imread(img,cv2.IMREAD_UNCHANGED)
    
    n_points_all = 0
    points = {(1,0):[], (-1,0):[]}    
    for c in range(img.shape[1]):
        for r in range(img.shape[0]):
            if img[r,c] > 0:
                points[(-1,0)].append({'pos_px':(c,r),'velocity':(-0.1,0,0),'duration':2.0})
                n_points_all = n_points_all + 1
                break

    for c in range(img.shape[1]):
        for r in range(img.shape[0]-1,-1,-1):
            if img[r,c] > 0:
                points[(1,0)].append({'pos_px':(c,r),'velocity':(0.1,0,0),'duration':2.0})
                n_points_all = n_points_all + 1
                break

    actions_res = {'train':[], 'test':[]}
    for name in actions_res:
        
        if name=='train':
            interval_px = n_points_all//10
        else:
            interval_px = n_points_all//18

        for key in points:
            n_points = len(points[key])
            n_contacts = n_points // interval_px
            i_beg = n_points//2 - (n_contacts-1)//2*interval_px
            i_end = n_points//2 + (n_contacts-1)//2*interval_px

            for i in range(i_beg,i_end+1,interval_px):
                actions_res[name].append(points[key][i])
    return actions_res

def get_setups_tools_sim(names, idxes_train, idxes_test, idx_param):

    for obj in names:
        fp_label = '../../dataset/objects/%s_label.pgm' % obj
        mass_minmax = [0.1, 5.0]
        
        setups = {'actor':{'name':'finger','type':'finger'},
                  'target_infos':[{'name':obj,
                                   'label':fp_label,
                                   'masses':params[obj][idx_param]['masses'],
                                   'meter_per_pixel':1/1000.0}],
                  'mass_minmax':mass_minmax,
                  'train':[],
                  'test':[]}
        
        actions = gen_actions_sim(fp_label)
        if idxes_train is None:
            idxes_train = range(len(actions['train']))
        for i in idxes_train:
            action = {'finger':{'pos_px':  actions['train'][i]['pos_px'],'yaw':0,
                                'velocity':actions['train'][i]['velocity'], 
                                'duration':actions['train'][i]['duration']},
                      'targets':[obj], obj:{'pos':[0,0], 'yaw':0}           }
            setups['train'].append(action)

        if idxes_test is None:
            idxes_test = range(len(actions['test']))
        for i in idxes_test:
            action = {'finger':{'pos_px':  actions['test'][i]['pos_px'],'yaw':0,
                                'velocity':actions['test'][i]['velocity'], 
                                'duration':actions['test'][i]['duration']},
                      'targets':[obj], obj:{'pos':[0,0], 'yaw':0}           }
            setups['test'].append(action)

    return setups

if __name__=='__main__':

    objects = ['book','box','cramp','crimp','hammer','ranch','snack','toothpaste','windex']
    for obj in objects:
        fp_label = '../../dataset/objects/%s_label.pgm' % obj
        img = cv2.imread(fp_label,cv2.IMREAD_UNCHANGED)

        actions = gen_actions_sim(fp_label)

        for i, name in enumerate(actions):
            plt.subplot(1,2,i+1)
            plt.imshow(img)
            for i, action in enumerate(actions[name]):
                plt.plot(action['pos_px'][0],action['pos_px'][1],'x')
                plt.text(action['pos_px'][0],action['pos_px'][1],str(i))
        plt.show()

params = {  'book':[ {'masses':[0.29281076, 4.17102208]},
                     {'masses':[0.41514522, 3.88697074]},
                     {'masses':[0.28492136, 4.24852271]},
                     {'masses':[0.20416621, 4.24841673]},
                     {'masses':[0.4764175 , 4.22357577]},
                     {'masses':[0.40250295, 4.15937436]},
                     {'masses':[0.13716004, 4.2290372 ]},
                     {'masses':[0.19650405, 3.76891762]},
                     {'masses':[0.39888124, 3.84463652]},
                     {'masses':[0.36953162, 4.04311101]} ],
             'box':[ {'masses':[4.84426277, 1.6408213 ]},
                     {'masses':[4.1998601 , 1.04142115]},
                     {'masses':[4.74380948, 1.11914328]},
                     {'masses':[4.87099288, 1.50774383]},
                     {'masses':[4.2941666 , 1.96757667]},
                     {'masses':[4.97265149, 1.53767697]},
                     {'masses':[4.07096144, 1.61055362]},
                     {'masses':[4.74469484, 1.53464246]},
                     {'masses':[4.61141535, 1.45047986]},
                     {'masses':[4.58190076, 1.2456486 ]} ],
          'cramp': [ {'masses':[0.11774639, 4.24031211]},
                     {'masses':[0.18171429, 4.88152415]},
                     {'masses':[0.16139963, 4.80959754]},
                     {'masses':[0.13699149, 4.14815628]},
                     {'masses':[0.1284138 , 4.69903566]},
                     {'masses':[0.13870274, 4.03959138]},
                     {'masses':[0.11061242, 4.64364075]},
                     {'masses':[0.14198293, 4.20037757]},
                     {'masses':[0.15152152, 4.68967399]},
                     {'masses':[0.19622997, 4.34173207]} ],
          'crimp': [ {'masses':[3.78247777, 1.45797222]},
                     {'masses':[3.61080564, 0.9885484 ]},
                     {'masses':[2.91861253, 0.5845852 ]},
                     {'masses':[2.98199777, 1.49202149]},
                     {'masses':[3.57327635, 0.54864304]},
                     {'masses':[3.63948487, 0.94408527]},
                     {'masses':[4.3242224 , 1.1353943 ]},
                     {'masses':[3.76296742, 0.7581765 ]},
                     {'masses':[4.13308188, 1.63447027]},
                     {'masses':[3.1492245 , 1.43725926]} ],
          'hammer':[ {'masses':[3.6708013 , 0.84426277]},
                     {'masses':[4.06142975, 1.1998601 ]},
                     {'masses':[3.31954728, 0.74380948]},
                     {'masses':[3.60764383, 0.87099288]},
                     {'masses':[3.85573177, 1.2941666 ]},
                     {'masses':[3.58376897, 0.97265149]},
                     {'masses':[3.710462  , 1.07096144]},
                     {'masses':[2.53462855, 0.74469484]},
                     {'masses':[3.75057986, 0.61141535]},
                     {'masses':[3.15606786, 1.58190076]} ],          
          'ranch': [ {'masses':[3.63828566, 0.00921507, 3.77555272]},
                     {'masses':[3.43726006, 0.00149763, 4.71774392]},
                     {'masses':[3.38798749, 0.00098332, 4.96525034]},
                     {'masses':[3.82219564, 0.00953866, 3.69913003]},
                     {'masses':[3.7792415 , 0.00181856, 3.61148054]},
                     {'masses':[2.30835818, 0.00811146, 4.40639678]},
                     {'masses':[4.53788377, 0.00493475, 4.44073034]},
                     {'masses':[3.9006551 , 0.00511314, 3.28042033]},
                     {'masses':[4.81628408, 0.00456034, 2.84560708]},
                     {'masses':[3.78062152, 0.00819118, 4.15879985]} ],
          'snack': [ {'masses':[1.11111264, 3.98672871]},
                     {'masses':[0.85822182, 3.95822135]},
                     {'masses':[1.06307084, 4.58427632]},
                     {'masses':[0.65132355, 4.07401829]},
                     {'masses':[0.87612745, 3.98209839]},
                     {'masses':[0.41527506, 4.27285216]},
                     {'masses':[1.28041014, 4.77507254]},
                     {'masses':[1.10087857, 3.97243879]},
                     {'masses':[0.93759184, 4.39030386]},
                     {'masses':[1.0415369 , 3.85477493]} ],
     'toothpaste': [ {'masses':[3.13637913, 1.40235665]},
                     {'masses':[3.77436965, 1.22316457]},
                     {'masses':[3.80144197, 1.05198367]},
                     {'masses':[3.73160204, 1.12904118]},
                     {'masses':[3.01175136, 1.75582657]},
                     {'masses':[3.88665794, 1.77340558]},
                     {'masses':[3.0408778 , 1.53924889]},
                     {'masses':[3.19007535, 1.18805848]},
                     {'masses':[3.95793507, 1.33816614]},
                     {'masses':[3.24363832, 1.6426988 ]} ],
         'windex': [ {'masses':[0.12221026, 4.13305229]},
                     {'masses':[0.17439786, 4.1272489 ]},
                     {'masses':[0.11469023, 4.27196908]},
                     {'masses':[0.10171519, 4.7654651 ]},
                     {'masses':[0.14768111, 4.50276396]},
                     {'masses':[0.14168922, 4.18832942]},
                     {'masses':[0.1252993 , 4.24386566]},
                     {'masses':[0.17358594, 4.93211186]},
                     {'masses':[0.11795237, 4.33754187]},
                     {'masses':[0.1964616 , 4.80574349]} ],
}
