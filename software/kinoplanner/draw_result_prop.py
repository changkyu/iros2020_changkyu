import os, sys
import pickle
import matplotlib
import matplotlib.colors as colors
import matplotlib.pyplot as plt
import numpy as np
import cv2
import pybullet as p

dir_pusher = os.path.abspath('..')
if os.path.abspath(dir_pusher) not in sys.path:
    sys.path.append(dir_pusher)
from simulator import SimBullet
from cellphysics.PhysicsBase import CellPhysicsBase
from plot_result_prop import palette

def set_color(sim, name, color):

    cellinfo = sim.getCellBodyProperties(name)
    for info in cellinfo:
        info['color'] = color
    sim.setCellBodyProperties(name,cellinfo)

methods = ['uniform','det','known','prop']
#objects = ['book','box','hammer','ranch','snack','windex']
objects = ['snack']


if __name__=='__main__':

    dir_res = './res_prop/'
    idx = 0
    meter_per_pixel = 0.53/640.0
    color_actor = [1,0.6470588235294118,0,1]
    dims_actor = [0.03, 0.03]

    for obj in objects:
        fp_label = '../../dataset/objects/%s_label.pgm' % obj

        succs_sel = []
        plans_sel = []
        for idx in range(100):
            succs = {}
            plans = {}
            for method in methods:
                if method=='det':
                    fp_res = os.path.join(dir_res,'%s1_%s_init%d.pkl'%(method,obj,idx))
                else:
                    fp_res = os.path.join(dir_res,'%s_%s_init%d.pkl'%(method,obj,idx))
                with open(fp_res,'rb') as f:
                    res = pickle.load(f)

                succs[method] = res['success']
                plans[method] = res['plan']

            if succs['prop']==False and \
               succs['det']==True and \
               succs['uniform']==False and\
               succs['known']==False:
                plans_sel.append(plans)
                succs_sel.append(succs)
                print(idx)

        sim = SimBullet(gui=True)        
        sim.setupCamera(   1.5,-90,-40, 0.10,0,0)
        sim.setupRendering(1.5,-90,-40,1000,1000,x_target=0.10)
        #sim.addObject('table_round','table_round',[1,1,1,1])
        #sim.setObjectPose('table_round',0.63,0,0)
        visId = sim._p.createVisualShape(p.GEOM_MESH, fileName='/home/cs1080/projects/iros2020/others/round_table.obj')
        sim._p.createMultiBody(0,-1, visId, [0.63,0,0])
        sim.addObject('finger','finger',[1,1,0,1])
        sim.setObjectPose('finger',10,0,0)
        cells = CellPhysicsBase.createCells(fp_label,[1,1,1,1],meter_per_pixel=meter_per_pixel)

        for succs, plans in zip(succs_sel,plans_sel):
            imgs = {}
            for method in methods:
                plan = plans[method]
                path = plan['path']
                actions = plan['actions']
                n_actions = len(actions)

                name = obj+'_'+method+'0'
                x,y,yaw = path[0]
                sim.addObject(name, cells, [0.5,0.5,0.5,1], x,y,yaw)
                color = list(colors.to_rgba(palette[method]))
                color[3] = 1#0.5
                set_color(sim, name, color)

                for i in range(n_actions):
                    """
                    action = actions[i]
                    for key, value in action.items() if type(action)==dict else action:
                        if key=='pos':
                            pos = [0,0]
                            pos[0] = np.cos(yaw)*value[0]-np.sin(yaw)*value[1] + x
                            pos[1] = np.sin(yaw)*value[0]+np.cos(yaw)*value[1] + y
                        elif key=='velocity':
                            velocity = [0,0,0]
                            velocity[0] = np.cos(yaw)*value[0]-np.sin(yaw)*value[1]
                            velocity[1] = np.sin(yaw)*value[0]+np.cos(yaw)*value[1]
                        elif key=='duration':
                            duration = value
                    length = np.linalg.norm(velocity)
                    sim.addObject('finger'+str(i+1),'finger',color_actor,
                        #pos[0]-velocity[0]/float(length)*dims_actor[0],\
                        #pos[1]-velocity[1]/float(length)*dims_actor[1],yaw)
                        pos[0]+velocity[0]*duration,\
                        pos[1]+velocity[1]*duration,yaw)
                    sim._p.addUserDebugLine([pos[0],pos[1],0.02], 
                                            [pos[0]+velocity[0]*duration,
                                             pos[1]+velocity[1]*duration,0.02])
                    """

                    name = obj+'_'+method+str(i+1)
                    x,y,yaw = path[i+1]
                    sim.addObject(name, cells, [0.5,0.5,0.5,1], x,y,yaw)
                    if i==n_actions-1 and succs[method]==False:
                        xyz,rot = sim.getObjectPose(name,ndims=3)
                        xyz = list(xyz)
                        xyz[2] = -1
                        sim._p.resetBasePositionAndOrientation(sim.objects[name]['bodyId'],xyz,[rot.q[1],rot.q[2],rot.q[3],rot.q[0]])
                        
                    color[3] = 1#0.5 + (i+1)/float(n_actions)
                    set_color(sim, name, color)

                img = sim.draw()
                imgs[method] = img

                for name in sim.objects:
                    if name.startswith(obj):
                        sim._p.removeBody(sim.objects[name]['bodyId'])
            
            for m, method in enumerate(methods):
                plt.subplot(1,len(methods),m+1)
                plt.imshow(imgs[method])
            plt.show()

            exit(0)