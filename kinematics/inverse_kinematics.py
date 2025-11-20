'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from numpy import atan2,sqrt,pi,acos
from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
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
        R = transform[0:3,0:3]
        posX = transform[0,3]
        posY = transform[1,3]
        posZ = transform[2,3]
        hipYawPitch = atan2(R[1,0], R[0,0])
        joint_angles.append(hipYawPitch)
        hipRoll = atan2(-posZ,posY)
        joint_angles.append(hipRoll)
        if effector_name == 'LLeg':
            _,_, KneePitchOffset = offsets['LKneePitch']
            _,_, AnklePitchOffset = offsets['LAnklePitch']
            KneePitchOffset = abs(KneePitchOffset)
            AnklePitchOffset = abs(AnklePitchOffset)
        elif effector_name == 'RLeg':
            _,_, KneePitchOffset = offsets['RKneePitch']
            _,_, AnklePitchOffset = offsets['RAnklePitch']
            KneePitchOffset = abs(KneePitchOffset)
            AnklePitchOffset = abs(AnklePitchOffset)
        dis = sqrt(posX**2 + posZ**2)
        cosLegKnee = (KneePitchOffset**2 + AnklePitchOffset**2-dis**2)/(2*KneePitchOffset*AnklePitchOffset)
        cosLegKnee = max(-1.0, min(1.0,cosLegKnee))
        legknee = pi -acos(cosLegKnee)
        joint_angles.append(legknee)
        cosHipNenner =(2*dis*KneePitchOffset)
        if cosHipNenner < 1*10**(-6):
            cosHipNenner=1.0
        else:
            cosHipNenner =(2*dis*KneePitchOffset)
        cosHip = (dis**2 + KneePitchOffset**2 - AnklePitchOffset**2)/cosHipNenner
        cosHip = max(-1.0, min(1.0,cosHip))
        hipPitch = atan2(posZ,posX)+ acos(cosHip)
        joint_angles.append(hipPitch)
        anklePitch = -(hipPitch+ legknee)
        joint_angles.append(anklePitch)
        ankleRoll = atan2(R[2,1], R[2,2])
        joint_angles.append(ankleRoll)
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        keyFramesAngles = self.inverse_kinematics(effector_name,transform)
        if effector_name== 'LLeg':
            keyFramesJoints = ['LHipYawPitch','LHipRoll','LHipPitch','LKneePitch','LAnklePitch','LAnkleRoll']
        elif effector_name == 'RLeg':
            keyFramesJoints = ['RHipYawPitch','RHipRoll','RHipPitch','RKneePitch','RAnklePitch','RAnkleRoll']
        else:
            keyFramesJoints = []
        times = []
        keys = []
        for i in range(len(keyFramesJoints)):
            times.append([0.0, 1.0]) 
            keys.append([
                [keyFramesAngles[i], [0,0,0], [0,0,0]], 
                [keyFramesAngles[i], [0,0,0], [0,0,0]]
            ])

        self.keyframes = (keyFramesJoints, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
