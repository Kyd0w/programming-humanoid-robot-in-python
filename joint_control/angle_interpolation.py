'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        names, times, keys = keyframes
        if not hasattr(self, "_motionStartTime"):
            self._motionStartTime = perception.time
        currentTime = perception.time - self._motionStartTime
        for k in range(len(names)):
            joint_name = names[k]
            jointTimes = times[k]
            jointKeys = keys[k]
            if len(jointTimes) == 0:
                continue
            if currentTime <= jointTimes[0]:
                target_joints[joint_name] = jointKeys[0][0]
                continue
            if currentTime >= jointTimes[-1]:
                target_joints[joint_name] = jointKeys[-1][0]
                continue
            t0 = t1 = None
            angle0 = angle1 = None
            h0_after = h1_before = None
            for j in range(len(jointTimes)-1):
                if jointTimes[j] <= currentTime < jointTimes[j+1]:
                    t0 = jointTimes[j]
                    t1 = jointTimes[j+1]

                    angle0, h0_before, h0_after = jointKeys[j]
                    angle1, h1_before, h1_after = jointKeys[j+1]
                    break
            if t0 is None:
                target_joints[joint_name] = jointKeys[-1][0]
                continue
            i = (currentTime - t0) / (t1 - t0)
            p0 = angle0
            p1 = angle0 + h0_after[2]
            p2 = angle1 + h1_before[2]
            p3 = angle1
            angle = (
                (1-i)**3 * p0
                + 3*(1-i)**2 * i * p1
                + 3*(1-i) * (i**2) * p2
                + (i**3) * p3
            )

            target_joints[joint_name] = angle
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
