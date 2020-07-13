import numpy as np
import pybullet as p

class SimBase:

    def __init__(self):
        self.shapes = self.get_shapes()

    def setupCamera(self, cam_dist=0.75, cam_yaw=90.0, cam_pitch=-90.0,
                          x_target=0, y_target=0, z_target=0, ):
        pass

    def addObject(self, name, shape, color=[0.5,0.5,0.5,1], x=0, y=0, yaw=0):
        pass

    def setObjectPose(self, name, x, y, yaw):
        pass

    def setCellBodyProperties(self, name, cells):
        pass

    def getObjectPose(self, name, ndims=2):
        pass

    def getCellBodyInfo(self, name):
        pass

    def applyConstantVelocity(self, name, velocity, duration):
        pass

    def stepSimulation(self):
        pass

    def draw(self):
        pass

    def addUserDebugText(self, name, pos, color, size, time, replaceItemUniqueId=-1):
        pass

    def recordStart(self, filename):
        pass

    def recordStop(self):
        pass

    @staticmethod
    def get_shapes():
        return\
        { 'plane':
          {'name':'plane',
           'type':'plane',
           'mass':0
          },
          'table_round':
          {'name':'table_round',
           'type':'table_round',
           'radius':0.5,
           'height':1,
           'z_offset':-0.500,
           'q_offset':[0,0,0,1],
           'mass':0,
           'fric':0
          },
          'table_tight':
          {'name':'table_tight',
           'type':'table_tight',
           'radius':0.25,
           'height':1,
           'z_offset':-0.500,
           'q_offset':[0,0,0,1],
           'mass':0,
           'fric':0
          },
          'table_rect':
          {'name':'table_rect',
           'type':'table_rect',
           'dims':[2,2,2],
           'z_offset':-1,
           'q_offset':[0,0,0,1],
           'mass':0,
           'fric':0
          },
          'finger':
          {'name':'finger',
           'type':'box',
           'mass':0,#10**6,
           'fric':0,
           'dims':[0.03,0.03,0.10],
           'z_offset':0.10*0.5,
           'q_offset':[0,0,0,1],
          },
          'soap':
          {'name':'soap',
           'type':'box',
           'mass':0.135,
           'dims':[0.035,0.066,0.096],
           'z_offset':0.035*0.5,
           'q_offset':p.getQuaternionFromEuler([0, -89.9999/ 180.0 * np.pi, 0]),
          },
        }
