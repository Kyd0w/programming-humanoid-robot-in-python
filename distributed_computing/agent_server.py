'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
import time
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler

from inverse_kinematics import InverseKinematicsAgent
import numpy as np

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent,self).__init__()
        self.start()
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint.get(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle
        return True

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE 
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes
        names, times, keys = keyframes
        max_t = max((lst[-1] for lst in times if lst), default=0.0)

        for t_list in times:
            if t_list:
                max_t = max(max_t, t_list[-1])
        startTime = self.perception.time
        while (self.perception.time - startTime) < max_t:
            time.sleep(0.01)
        return True


    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        matrix = self.transforms.get(name)
        if matrix is None:
            return None
        return [[float(matrix[i, j]) for j in range(4)] for i in range(4)]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        matrix = np.array(transform)
        self.set_transforms(effector_name, matrix)
        return True

class RequestHander(SimpleXMLRPCRequestHandler):
    rpc_paths =('/RPC2',)

if __name__ == '__main__':
    agent = ServerAgent()
    server = SimpleXMLRPCServer(
        ('localhost', 8000),
        requestHandler= RequestHander,
        allow_none=True,
        logRequests=False
    )
    server.register_instance(agent)
    server.serve_forever()

