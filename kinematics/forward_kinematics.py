'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity, cos, sin

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LeftArm':['LShoulderPitch','LShoulderRoll','LElbowYaw','LElbowRoll'],
                       'RightArm':['RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll'],
                       'LeftLeg':['LHipYawPitch','LHipRoll','LHipPitch','LKneePitch','LAnklePitch','LAnkleRoll'],
                       'RightLeg':['RHipYawPitch','RHipRoll','RHipPitch','RKneePitch','RAnklePitch','RAnkleRoll']
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        axis = ''
        if joint_name in ['HeadYaw','LElbowYaw','RElbowYaw']:
            axis = 'z'
        elif joint_name in ['HeadPitch',
                            'LShoulderPitch',
                            'LHipPitch','LKneePitch','LAnklePitch',
                            'RShoulderPitch',
                            'RHipPitch','RKneePitch','RAnklePitch']:
            axis = 'y'
        elif joint_name in ['LShoulderRoll','LElbowRoll',
                            'RShoulderRoll','RElbowRoll',
                            'LHipRoll','LAnkleRoll',
                            'RHipRoll','RAnkleRoll'
                            ]:
            axis = 'x'
        elif joint_name in ['LHipYawPitch','RHipYawPitch']:
            axis = 'zy'
        else:
            axis = 'null'
        c= cos(joint_angle)
        s= sin(joint_angle)
        Tr = identity(3)
        offsets = {
            'HeadYaw':          ( 0.0, 0.0, 0.1265),
            'HeadPitch':        ( 0.0, 0.0, 0.0),
            'LShoulderPitch':   ( 0.0, 0.098,0.1),
            'LShoulderRoll':    ( 0.0, 0.0, 0.0),
            'LElbowYaw':        ( 0.105, 0.015, 0.0),
            'LElbowRoll':       ( 0.0, 0.0, 0.0),
            'RShoulderPitch':   ( 0.0, -0.098,0.1),
            'RShoulderRoll':    ( 0.0, 0.0, 0.0),
            'RElbowYaw':        ( 0.105,-0.015, 0.0),
            'RElbowRoll':       ( 0.0, 0.0, 0.0),
            'LHipYawPitch':     ( 0.0, 0.05, -0.085),
            'LHipRoll':         ( 0.0, 0.0, 0.0),
            'LHipPitch':        ( 0.0, 0.0, 0.0),
            'LKneePitch':       ( 0.0, 0.0, -0.1),
            'LAnklePitch':      ( 0.0, 0.0, -0.1029),
            'LAnkleRoll':       ( 0.0, 0.0, 0.0),
            'RHipYawPitch':     ( 0.0, -0.05, -0.085),
            'RHipRoll':         ( 0.0, 0.0, 0.0),
            'RHipPitch':        ( 0.0, 0.0, 0.0),
            'RKneePitch':       ( 0.0, 0.0, -0.1),
            'RAnklePitch':      ( 0.0, 0.0, -0.1029),
            'RAnkleRoll':       ( 0.0, 0.0, 0.0)
        }  
        offx,offy,offz = offsets[joint_name]
        match axis:
            case 'z':
                Tr = matrix([
                    [ c,-s, 0],
                    [ s, c, 0],
                    [ 0, 0, 1]
                ])
            case 'y':
                Tr = matrix([
                    [ c, 0, s],
                    [ 0, 1, 0],
                    [-s, 0, c]
                ])
            case 'x':
                Tr = matrix([
                    [ 1, 0, 0],
                    [ 0, c,-s],
                    [ 0, s, c]
                ])
            case 'zy':
                Tr = matrix([
                    [ c,-s, 0],
                    [ s, c, 0],
                    [ 0, 0, 1]
                ]) * matrix([
                    [ c, 0, s],
                    [ 0, 1, 0],
                    [-s, 0, c],
                ])
            case _:
                Tr = identity(3)

        T = matrix([
            [Tr[0,0],Tr[0,1],Tr[0,2],offx],
            [Tr[1,0],Tr[1,1],Tr[1,2],offy],
            [Tr[2,0],Tr[2,1],Tr[2,2],offz],
            [ 0,    0,  0,  1]
        ])    
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T*Tl
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
