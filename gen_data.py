import numpy as np

# landmarks = np.array([3, 6, 3, 12, 7, 8, 7, 14, 11, 6, 11, 12], dtype=float)
#landmarks = [[3, 6],[] 3, 12, 7, 8, 7, 14, 11, 6, 11, 12]
landmarks = [[3,6],[3,12],[7,8],[7,14],[11,6],[11,12]]
#start =  np.array([0,0])
curr = [0,0,0]
cmds = [[3,0],[3,0],[3,0], [3,0], [3,0], [1.0, 1.2566],
        [3,0],[3,0],[3,0], [3,0], [3,0], [1.0, 1.2566],
        [3,0],[3,0],[3,0], [3,0], [3,0], [1.0, 1.2566],
        [3,0],[3,0],[3,0], [3,0], [3,0], [1.0, 1.2566],
        [3,0],[3,0],[3,0], [3,0], [3,0]]

cmds = [[3,0],[3,0],[3,0], [1,0], [3,0], [1.0, 1.57],
        [3,0],[3,0],[3,0], [1,0], [3,0], [1.0, 1.57],
        [3,0],[3,0],[3,0], [1,0], [3,0], [1.0, 1.57],
        [3,0],[3,0],[3,0], [1,0], [3,0], [1.0, 0],
        [3,0],[3,0],[3,0], [1,0], [3,0]]

sigd = 0.002
sigth = 0.002
sigr = 0.04
sigbeta = 0.04
data = []
def calcReading(pos, landmark):
    r = np.linalg.norm([landmark[0]-pos[0], landmark[1] - pos[1]])
    th = np.arctan2(landmark[1] - pos[1], landmark[0] - pos[0])
    th_r = warp2pi(th - pos[2])
    r += np.random.normal(0,sigr)
    th_r += np.random.normal(0,sigbeta)
    return[th_r, r]

def update(pos, cmd):
    d = cmd[0]
    d += np.random.normal(0,sigd)
    pos[0] += d * np.cos(pos[2])
    pos[1] += d * np.sin(pos[2])
    pos[2] += cmd[1] + np.random.normal(0,sigth)
    pos[2] = warp2pi(pos[2])
    return pos

def warp2pi(angle_rad):
    """
    TODO: warps an angle in [-pi, pi]. Used in the update step.

    \param angle_rad Input angle in radius
    \return angle_rad_warped Warped angle to [-\pi, \pi].
    """
    angle_rad = angle_rad % (2 * np.pi)#wrap to 2pi
    if(angle_rad > np.pi):
        angle_rad = -2 * np.pi + angle_rad
    return angle_rad

for i, cmd in enumerate(cmds):
    readings = []
    for i in range(len(landmarks)):
        readings.extend(calcReading(curr, landmarks[i]))
    data.append(readings)
    data.append(cmd)
    curr = update(curr, cmd)
readings = []
for i in range(len(landmarks)):
    readings.extend(calcReading(curr, landmarks[i]))
data.append(readings)

print(data)

with open("gen_datab1.txt", 'w') as file:
    for l in data:
        line = "\t".join(format(x, ".4f") for x in l)
        file.write(line + "\t\n")
pass    

