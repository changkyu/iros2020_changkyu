import time
import math
import numpy as np

import pybullet as p
from pybullet_utils import bullet_client

from pyquaternion import quaternion
from SimBase import SimBase

def quaterion_to_euler(quat):
    if type(quat)=='list':
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]
    else:
        w = quat.q[0]
        x = quat.q[1]
        y = quat.q[2]
        z = quat.q[3]            
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)

    X = math.atan2(t0, t1)
    Y = math.asin(t2)
    Z = math.atan2(t3, t4)

    return (X,Y,Z)

def distance_angle(b, a):

    alpha = a * 180.0 / np.pi;
    beta  = b * 180.0 / np.pi;
    d = (beta - alpha) % 360;    
    if d > 180:
        res = -360 + d;
    elif d < -180:
        res =  360 + d;
    else:
        res = d;

    return res * np.pi / 180.0;

class SimBullet(SimBase):

    def __init__(self, gui=False):

        SimBase.__init__(self)

        if gui:
            self._p = bullet_client.BulletClient(p.GUI)
            self._p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            self._p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        else:
            self._p = bullet_client.BulletClient(p.DIRECT)
        
        self._p.setGravity(0,0,-9.8)
        

        for name in self.shapes:
            self.shapes[name]['colId'] = self.createCollisionShape(name)

        self.dt = self._p.getPhysicsEngineParameters()['fixedTimeStep']

        # Rendering
        self.setupRendering()
        self.objects = {}        
    
    def setupRendering(self, cam_dist=0.75, cam_yaw=-90.0, cam_pitch=-89.99, 
                             render_width=244, render_height=244,
                             x_target=0, y_target=0, z_target=0 ):
        
        self._cam_dist = cam_dist
        self._cam_yaw = cam_yaw
        self._cam_pitch = cam_pitch
        self._render_width = render_width
        self._render_height = render_height
        
        self.view_matrix = self._p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[x_target,y_target,z_target],
                                                                     distance=self._cam_dist,
                                                                     yaw=self._cam_yaw,
                                                                     pitch=self._cam_pitch,
                                                                     roll=0,
                                                                     upAxisIndex=2)
        self.proj_matrix = self._p.computeProjectionMatrixFOV(fov=53.130,
                                                              aspect=float(self._render_width) / self._render_height,
                                                              nearVal=0.001,
                                                              farVal=100.0)

    def setupCamera(self, cam_dist=0.75, cam_yaw=90.0, cam_pitch=-90.0,
                          x_target=0, y_target=0, z_target=0, ):
        self._p.resetDebugVisualizerCamera(cam_dist,cam_yaw,cam_pitch,
                                           [x_target,y_target,z_target])

    def createCollisionShape(self, name):
        shape_type = self.shapes[name]['type']
        if shape_type=='plane':            
            return self._p.createCollisionShape(p.GEOM_PLANE)
        elif shape_type=='box':
            return self._p.createCollisionShape(p.GEOM_BOX,
                                                halfExtents=list(np.array(self.shapes[name]['dims'])*0.5))
        elif shape_type=='table_rect':
            return self._p.createCollisionShape(p.GEOM_BOX,
                                                halfExtents=list(np.array(self.shapes[name]['dims'])*0.5))
        elif shape_type=='table_round' or shape_type=='table_tight':
            return self._p.createCollisionShape(p.GEOM_CYLINDER,
                                                radius=self.shapes[name]['radius'],
                                                height=self.shapes[name]['height'])
        elif shape_type=='mesh':
            return self._p.createCollisionShape(p.GEOM_MESH,
                                                fileName=self.shapes[name]['mesh'])
        else:
            raise NotImplementedError("%s shape type is not implemented yet" % shape_type)

    def createBody(self, name, color, x=0, y=0, yaw=0):

        shape_type = self.shapes[name]['type']
        if shape_type=='plane':
            #visId = self._p.createVisualShape( shapeType=p.GEOM_PLANE, rgbaColor=color )
            visId = -1
            bodyId = self._p.createMultiBody( self.shapes[name]['mass'],
                                              self.shapes[name]['colId'], visId, 
                                              [0,0,0] )
        elif shape_type=='table_round' or shape_type=='table_tight':
            visId = self._p.createVisualShape( shapeType=p.GEOM_CYLINDER,
                                               radius=self.shapes[name]['radius'], 
                                               length=self.shapes[name]['height'], 
                                               rgbaColor=color )
            bodyId = self._p.createMultiBody( self.shapes[name]['mass'],
                                              self.shapes[name]['colId'], visId, 
                                              [x,y,self.shapes[name]['z_offset']] )
        elif shape_type=='table_rect':
            visId = self._p.createVisualShape( shapeType=p.GEOM_BOX, 
                                               halfExtents=list(np.array(self.shapes[name]['dims'])*0.5),
                                               rgbaColor=color )
            bodyId = self._p.createMultiBody(  self.shapes[name]['mass'],
                                               self.shapes[name]['colId'], visId, 
                                               [x,y,self.shapes[name]['z_offset']] )
        elif shape_type=='finger':
            pos, rot = self.getPosRot(self.shapes[name],x,y,yaw)
            visId = self._p.createVisualShape( shapeType=p.GEOM_BOX, 
                                               halfExtents=list(np.array(self.shapes[name]['dims'])*0.5),
                                               rgbaColor=color )
            bodyId = self._p.createMultiBody( self.shapes[name]['mass'],
                                              self.shapes[name]['colId'], visId, 
                                              pos, rot )
        elif shape_type=='box':
            pos, rot = self.getPosRot(self.shapes[name],x,y,yaw)
            visId = self._p.createVisualShape( shapeType=p.GEOM_BOX, 
                                               halfExtents=list(np.array(self.shapes[name]['dims'])*0.5),
                                               rgbaColor=color )
            bodyId = self._p.createMultiBody(  self.shapes[name]['mass'],
                                               self.shapes[name]['colId'], visId, 
                                               pos, rot                          )
        elif shape_type=='mesh':
            pos, rot = self.getPosRot(self.shapes[name],x,y,yaw)
            visId = self._p.createVisualShape( p.GEOM_MESH,
                                               fileName=self.shapes[name]['mesh'],
                                               rgbaColor=color )
            bodyId = self._p.createMultiBody(  self.shapes[name]['mass'],
                                               self.shapes[name]['colId'], visId, 
                                               pos, rot                          )
        else:
            raise NotImplementedError("%s shape type is not implemented yet" % shape_type)

        #self._p.changeDynamics(bodyId,-1,linearDamping=0.99,angularDamping=0.99)
        self._p.changeDynamics(bodyId,-1,lateralFriction=2.0)
        return visId, bodyId

    def createCellBody(self, cells, cell_size, color, x=0, y=0, yaw=0):

        dims = [cell_size]*3
        colId = self._p.createCollisionShape( p.GEOM_BOX, halfExtents=dims )
        visId = self._p.createVisualShape( shapeType=p.GEOM_BOX, 
                                           halfExtents=dims, rgbaColor=color)
        
        pos = [x,y,dims[2]*0.5]
        rot = self._p.getQuaternionFromEuler([0,0,yaw])
        n_cells = len(cells)

        cellpos = []
        cellrot = []
        inpos = []
        inrot = []
        colIds = []
        visIds = []
        masses = []
        for cell in cells:
            colIds.append(colId)
            visIds.append(visId)
            masses.append(cell['mass'])
            cellpos.append([cell['pos'][0],cell['pos'][1],dims[2]*0.5])
            #inpos.append([cell['pos'][0]-x,cell['pos'][1]-y,0])
            inpos.append([0,0,0])
            cellrot.append([0,0,0,1])
            inrot.append([0,0,0,1])

        bodyId = self._p.createMultiBody( baseMass=0.0000001,
                                          basePosition=pos,
                                          baseOrientation=rot,
                                          linkMasses=[1]*n_cells,
                                          linkCollisionShapeIndices=[colId]*n_cells,
                                          linkVisualShapeIndices=[visId]*n_cells,
                                          linkPositions=cellpos,
                                          linkOrientations=cellrot,
                                          linkInertialFramePositions=inpos,
                                          linkInertialFrameOrientations=inrot,
                                          linkParentIndices=[0]*n_cells,
                                          linkJointTypes=[p.JOINT_FIXED]*n_cells,
                                          linkJointAxis=[[0,0,1]]*n_cells
                                        )

        for c, cell in enumerate(cells):
            self._p.changeDynamics(bodyId,c,mass=cell['mass'])

        #self._p.changeDynamics(bodyId,-1,linearDamping=0.99,angularDamping=0.99)
        self._p.changeDynamics(bodyId,-1,lateralFriction=2.0)
        return bodyId

    def addObject(self, name, shape, color=[0.5,0.5,0.5,1], x=0, y=0, yaw=0):

        if len(color)==3:
            color = list(color) + [1,]

        if type(shape)==str:
            shape = self.shapes[shape]
            visId, bodyId = self.createBody(shape['name'],color, x,y,yaw )
        elif type(shape)==dict:
            bodyId = self.createCellBody(shape['cells'],shape['cell_size'],color, x,y,yaw )
            visId = -1

            pos,rot = self._p.getBasePositionAndOrientation(bodyId)
            shape = {'name':name, 'type':'cell', 
                     'z_offset':pos[2], 'q_offset':[0,0,0,1],
                     'cells': shape['cells'], 
                     'cell_size': shape['cell_size'],
                     'group:':shape['group'],
                     'cell2group':shape['cell2group'], 
                     'group2cell':shape['group2cell'],
                    }
            self.shapes[name] = shape
        else:
            NotImplementedError("%s is not implemented yet" % type(shape))

        self.objects[name] = {'shape':shape, 'color':color, 'visId':visId, 'bodyId':bodyId}

    def getPosRot(self, shape, x, y, yaw):

        z_offset = shape['z_offset']
        q_offset = shape['q_offset']
        q_offset = quaternion.Quaternion(x=q_offset[0], y=q_offset[1], z=q_offset[2], w=q_offset[3]).normalised

        pos = [x,y,z_offset]
        q_delta = self._p.getQuaternionFromEuler([0,0,yaw])
        q_delta = quaternion.Quaternion(x=q_delta[0], y=q_delta[1], z=q_delta[2], w=q_delta[3]).normalised
        rot = q_delta * q_offset
        rot = list(rot.q[1:])+ [rot.q[0]]

        return pos, rot

    def setObjectPose(self, name, x, y, yaw):

        pos, rot = self.getPosRot(self.objects[name]['shape'], x, y, yaw)
        self._p.resetBasePositionAndOrientation(self.objects[name]['bodyId'],pos,rot)

    def getObjectPose(self, name, ndims=2):
        
        bodyId = self.objects[name]['bodyId']
        pos,rot = self._p.getBasePositionAndOrientation(bodyId)

        if ndims==2:
            q_offset = self.objects[name]['shape']['q_offset']
            q_offset = quaternion.Quaternion(x=q_offset[0],y=q_offset[1],z=q_offset[2],w=q_offset[3]).normalised
            q_rot = quaternion.Quaternion(x=rot[0],y=rot[1],z=rot[2],w=rot[3]).normalised
            q_rot = (q_rot * q_offset.inverse).normalised

            (_,_,yaw) = quaterion_to_euler(q_rot)

            x = pos[0]
            y = pos[1]
            yaw = distance_angle(yaw,0)

            return x,y,yaw
        else:
            q_rot = quaternion.Quaternion(x=rot[0],y=rot[1],z=rot[2],w=rot[3]).normalised
            return pos,q_rot

    def isObjectCollide(self, name):

        bodyId = self.objects[name]['bodyId']
        for name_other in self.objects:
            bodyId_other = self.objects[name_other]['bodyId']
            if bodyId==bodyId_other:
                continue
            points = self._p.getClosestPoints(bodyId, bodyId_other)
            if len(points) > 0:
                return True        
        return False

    def getCellBodyPose(self, name):

        x_base, y_base, yaw_base = self.getObjectPose(name)
        cells = self.objects[name]['shape']['cells']
        pos_cells = []
        for cell in cells:
            pos = cell['pos']
            x = (pos[0]*np.cos(yaw_base)-pos[1]*np.sin(yaw_base)) + x_base
            y = (pos[0]*np.sin(yaw_base)+pos[1]*np.cos(yaw_base)) + y_base
            pos_cells.append([yaw_base, x,y])
        return pos_cells

    def getCellBodyVelocity(self, name):

        bodyId = self.objects[name]['bodyId']
        states = self.getLinkStates(bodyId, computeLinkVelocity=True)
        ret = []
        for state in states:
            vel_lin = state[6]
            vel_ang = state[7]
            ret.append([vel_ang[2],vel_lin[0],vel_lin[1]])
        return ret

    def setObjectProperties(self, name, mass, friction):

        bodyId = self.objects[name]['bodyId']
        self._p.changeDynamics(bodyId,-1,mass=mass, lateralFriction=friction)

    def setCellBodyProperties(self, name, props):

        bodyId = self.objects[name]['bodyId']
        for c, prop in enumerate(props):
            if 'mass' in prop:
                self._p.changeDynamics(bodyId,c,mass=prop['mass'])
            if 'fric' in prop:
                self._p.changeDynamics(bodyId,c,lateralFriction=prop['fric'])
            if 'color' in prop:
                self._p.changeVisualShape(bodyId,c,rgbaColor=prop['color'])

    def getCellBodyProperties(self, name):

        bodyId = self.objects[name]['bodyId']
        cells = self.objects[name]['shape']['cells']
        properties = []
        for c in range(len(cells)):
            info = self._p.getDynamicsInfo(bodyId,c)            
            properties.append({'mass':info[0], 'fric':info[1]})
        return properties

    def move_and_rotate(self, name, pos_delta, yaw_delta):

        bodyId = self.objects[name]['bodyId']
        
        pos_delta = np.array(pos_delta)
        rot_delta = self._p.getQuaternionFromEuler([0,0,yaw_delta])
        rot_delta = quaternion.Quaternion(x=rot_delta[0], y=rot_delta[1], z=rot_delta[2], w=rot_delta[3]).normalised
        
        pos_curr, rot_curr = self._p.getBasePositionAndOrientation(bodyId)
        pos_curr = np.array(pos_curr)
        rot_curr = quaternion.Quaternion(x=rot_curr[0], y=rot_curr[1], z=rot_curr[2], w=rot_curr[3]).normalised

        pos_goal = pos_delta + pos_curr
        rot_goal = rot_delta * rot_curr
        rot_goal = rot_goal.normalised

        # 20 degree per second OR 3cm per second
        dist = np.linalg.norm(pos_delta)
        time_pos = dist / 0.01;
        time_rot = np.abs(yaw_delta) * (180.0/20.0) / np.pi;
        time_req = max(max(time_pos,time_rot),0.1)

        time_delta = 0.1;
        n_iters = np.round(time_req / time_delta)

        pos_per_iter = list(np.array(pos_delta)/float(n_iters))
        rot_per_iter = self._p.getQuaternionFromEuler([0,0,yaw_delta/float(n_iters)])
        rot_per_iter = quaternion.Quaternion(x=rot_per_iter[0], y=rot_per_iter[1], z=rot_per_iter[2], w=rot_per_iter[3]).normalised

        for i in range(int(n_iters)):
            pos_curr = pos_per_iter + pos_curr
            rot_curr = rot_per_iter * rot_curr
            rot_curr = rot_curr.normalised

            rot = list(rot_curr.q[1:])+ [rot_curr.q[0]]
            self._p.resetBasePositionAndOrientation(bodyId, pos_curr, rot )
            self._p.stepSimulation()
            time.sleep(0.01)

    def applyExternalForce(self, name, force, duration):

        bodyId = self.objects[name]['bodyId']
        pos,rot = self._p.getBasePositionAndOrientation(bodyId)
                
        #import cv2

        self._p.applyExternalForce(bodyId, -1, force, pos, p.WORLD_FRAME)

        for i in range(int(duration/self.dt)):            
            self._p.stepSimulation()

            #if i % 24==0:
            #    pos,rot = self._p.getBasePositionAndOrientation(bodyId)
            #    print(pos)
            #    img = self.draw()
            #    cv2.imshow('debug',cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            #    cv2.waitKey()

    def applyConstantVelocity(self, name, velocity, duration, traj_names=None, traj_ref=None):

        i_ref = 0
        n_iters = int(duration/self.dt)
        n_iters_traj = int(0.1/self.dt)
        
        if traj_names is not None:
            trajectory = {}
            for obj in traj_names:
                trajectory[obj] = {'pos':[], 'cellpos':[]}

            bodyId = self.objects[name]['bodyId']
            self._p.stepSimulation()
            for i in range(n_iters):
                self._p.resetBaseVelocity(bodyId, linearVelocity=velocity)
                self._p.stepSimulation()

                if i % n_iters_traj==0:
                    for obj in traj_names:
                        x,y,yaw = self.getObjectPose(obj)
                        trajectory[obj]['pos'].append((x,y,yaw))
                        if self.objects[obj]['shape']['type']=='cell':
                            cellpos = self.getCellBodyPose(obj)
                            trajectory[obj]['cellpos'].append(cellpos)

                    if traj_ref is not None:
                        for obj in traj_ref:
                            if i_ref < len(traj_ref[obj]['pos']):
                                x,y,yaw = traj_ref[obj]['pos'][i_ref]
                                self.setObjectPose(obj,x,y,yaw)
                        i_ref = i_ref + 1

            self._p.resetBaseVelocity(bodyId, linearVelocity=[0,0,0])
            return trajectory
        else:
            bodyId = self.objects[name]['bodyId']
            self._p.stepSimulation()
            for i in range(n_iters):
                self._p.resetBaseVelocity(bodyId, linearVelocity=velocity)
                self._p.stepSimulation()
            self._p.resetBaseVelocity(bodyId, linearVelocity=[0,0,0])

        for i in range(100):
            self._p.stepSimulation()

        #
        #bodyId = self.objects[name]['bodyId']
        #self._p.stepSimulation()
        #for i in range(int(duration/self.dt)):
        #    self._p.resetBaseVelocity(bodyId, linearVelocity=velocity)
        #    self._p.stepSimulation()
        #self._p.resetBaseVelocity(bodyId, linearVelocity=[0,0,0])
            #for obj in self.objects:
            #    if obj not in ['plane']:
            #        x,y,yaw = self.getObjectPose(obj)
            #        trajectory.append({'name':obj, 'pos':(x,y,yaw)})
            #self._p.stepSimulation()
        #return trajectory

    def stepSimulation(self):
        self._p.stepSimulation()

    def draw(self):

        (_, _, px, _, _) = self._p.getCameraImage(width=self._render_width,
                                                  height=self._render_height,
                                                  viewMatrix=self.view_matrix,
                                                  projectionMatrix=self.proj_matrix,
                                                  renderer=p.ER_BULLET_HARDWARE_OPENGL)
        img = np.array(px)
        img = np.reshape(np.array(px), (self._render_height, self._render_width, -1))
        img = img[:,:,:3].copy()

        return img

    def addUserDebugText(self, text, pos, color, size, time, replaceItemUniqueId=-1):
        return self._p.addUserDebugText(text, pos, color, size, time, replaceItemUniqueId=replaceItemUniqueId)

    def recordStart(self, filename):
        self._p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, fileName=filename)

    def recordStop(self):
        self._p.stopStateLogging()
