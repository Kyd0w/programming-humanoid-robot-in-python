'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpc.client
import threading
import numpy as np

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.execute_keyframes, args=(keyframes,))
        thread.daemon = True
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.set_transform, args=(effector_name, transform))
        thread.daemon = True
        thread.start()



class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''

    # YOUR CODE HERE
    def __init__(self):
        self.server = xmlrpc.client.ServerProxy('http://localhost:8000',allow_none=True)
        self.post = PostHandler(self)
        
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        angle = self.server.get_angle(joint_name)
        return angle


    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        
        return self.server.set_angle(joint_name, angle)
        

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        posture = self.server.get_posture()
        return posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        
        return self.server.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        transformed = self.server.get_transform(name)
        return transformed

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        
        return self.server.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    print("get_angle(Headyaw)")
    angle = agent.get_angle("HeadYaw")
    print("angle"+ str(angle))

    print("set_angle")
    result = agent.set_angle("HeadYaw", 0.3)
    print(result)

    print("get_prosture")
    result = agent.get_posture()
    print(result)

    print("execute_keyFrames")
    names = ["HeadYaw"]
    times = [[0.0, 1.0]]
    keys = [
        [
            [0.0, [0, 0.0, 0.0], [0, 0.0, 0.0]],
            [0.5, [0, 0.0, 0.0], [0, 0.0, 0.0]]
        ]
    ]
    keyframes = (names, times, keys)
    agent.execute_keyframes(keyframes)

    print("get_transform")
    result = agent.get_transform("HeadYaw")
    print(result)

    print("set_tranform")
    testMatrix = np.eye(4)
    testMatrix[0, 3] = 0.01
    testMatrix[1, 3] = -0.02  
    testMatrix[2, 3] = -0.10  
    test_list = [[float(testMatrix[i,j]) for j in range(4)] for i in range(4)]
    result = agent.set_transform("LLeg", test_list)
    print(result)

    
