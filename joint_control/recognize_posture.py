'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


import os
from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        stupidPath = os.path.join(os.path.dirname(__file__), "robot_pose.pkl")
        self.posture_classifier = pickle.load(open(stupidPath, "rb"))# LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        classes = ['Back', 'Belly', 'Crouch', 'Frog', 'HeadBack', 'Knee', 'Left', 'Right', 'Sit', 'Stand', 'StandInit']
        joint_names = ['HeadPitch','LHipYawPitch', 'LHipRoll', 'LHipPitch','LKneePitch', 'LAnklePitch', 'LAnkleRoll','RHipPitch', 'RKneePitch', 'RAnklePitch']
        features = []
        for j in joint_names:
            value = perception.joint.get(j, 0.0)
            features.append(value)    
        pred_id = self.posture_classifier.predict([features])[0]
        posture = classes[pred_id]
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
