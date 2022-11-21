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

def bezier_interpolation(t, k0, k1, k2, k3):
    b = (1-t)**3 * k0 + 3 * t * (1-t)**2 * k1 + 3 * t**2 * (1-t) * k2 + t**3 * k3
    return b


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.begin = -1
        self.keyframes = ([], [], [])
       

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)



    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        if(self.keyframes == ([],[],[])):
            self.begin = perception.time
            return target_joints
        if (self.begin < 0):
            self.begin = perception.time

        time_since_begin = perception.time - self.begin
 
        
        names, times, keys = keyframes

    
        for curr_joint in range(len(names)):
            name = names[curr_joint]
            if (name not in self.joint_names):
                continue
            joint_keys = keys[curr_joint]
            joint_times = times[curr_joint]
            
                  
            for curr_time in range(len(joint_times)-1):
                t0 = joint_times[curr_time]
                t1 = joint_times[curr_time+1]

                if (t0 < time_since_begin < t1):
                    t = (time_since_begin - t0) / (t1 - t0)
                    k0 = joint_keys[curr_time][0]
                    k3 = joint_keys[curr_time+1][0]
                    k1 = k0 + joint_keys[curr_time][1][2]
                    k2 = k3 + joint_keys[curr_time][2][2]
                    target_joints[name] = bezier_interpolation(t, k0, k1, k2, k3)
                    if (name == 'LHipYawPitch'):
                        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
