'''
    Initially written by Ming Hsiao in MATLAB
    Adapted to Python by Akash Sharma (akashsharma@cmu.edu), 2020
    Updated by Wei Dong (weidong@andrew.cmu.edu), 2021
'''


'''Plan for Multi-Agent EKF:
Let's start with just 2 agents and fix the number of landmarks to be 6 (just like HW2).
Predict step is still independent (motion model of each individual robot) - should have size 3 + 2k for k landmarks.
Update step is where we will need to combine the information from different agents - can take weighted sum of landmark 
positions and use that for Jacobians, or can change covariance matrices (see midterm report).
'''


import numpy as np
import math
import re
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True, threshold=np.inf, linewidth=np.inf)
def _h (X, k):
    
    # extract pose x y theta
    x, y, theta = X[:3].ravel()
    
    # initialize measurement function h
    h = np.zeros((2 * k, 1))
    
    # loop calculation for each landmark
    for i in range(k):
        # landmark position
        lx, ly = X[3 + 2 * i:3 + 2 * i + 2].ravel()
        
        # calculate the expected landmark measurements
        dx = lx - x
        dy = ly - y
        pow_dxdy = dx**2 + dy**2
        sqr_dxdy = np.sqrt(pow_dxdy)
        b_hat = np.arctan2(dy, dx) - theta
        r_hat = sqr_dxdy
        
        # update the measurement function h
        h[2 * i] = warp2pi(b_hat)
        h[2 * i + 1] = r_hat

    return h

def draw_cov_ellipse(mu, cov, color):
    """
    Draws an ellipse in plt canvas.

    \param mu Mean of a Gaussian
    \param cov Covariance of a Gaussian
    \param color Color in plt format, e.g. 'b' for blue, 'r' for red.
    """
    U, s, Vh = np.linalg.svd(cov)
    a, b = s[0], s[1]
    vx, vy = U[0, 0], U[0, 1]
    theta = np.arctan2(vy, vx)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    phi = np.arange(0, 2 * np.pi, np.pi / 50)
    rot = []
    for i in range(100):
        rect = (np.array(
            [3 * np.sqrt(a) * np.cos(phi[i]),
             3 * np.sqrt(b) * np.sin(phi[i])]))[:, None]
        rot.append(R @ rect + mu)

    rot = np.asarray(rot)
    plt.plot(rot[:, 0], rot[:, 1], c=color, linewidth=0.75)


def draw_traj_and_pred(X, P):
    """ Draw trajectory for Predicted state and Covariance

    :X: Prediction vector
    :P: Prediction Covariance matrix
    :returns: None

    """
    draw_cov_ellipse(X[0:2], P[0:2, 0:2], 'm')
    plt.draw()


def draw_traj_and_map(X, last_X, P, t):
    """Draw Trajectory and map

    :X: Current state
    :last_X: Previous state
    :P: Covariance
    :t: timestep
    :returns: None

    """
    plt.ion()
    draw_cov_ellipse(X[0:2], P[0:2, 0:2], 'b')
    plt.plot([last_X[0], X[0]], [last_X[1], X[1]], c='b', linewidth=0.75)
    plt.plot(X[0], X[1], '*b')

    if t == 0:
        for k in range(6):
            draw_cov_ellipse(
                X[3 + k * 2:3 + k * 2 + 2], P[3 + k * 2:3 + 2 * k + 2,
                                              3 + 2 * k:3 + 2 * k + 2], 'r')
    else:
        for k in range(6):
            draw_cov_ellipse(
                X[3 + k * 2:3 + k * 2 + 2], P[3 + 2 * k:3 + 2 * k + 2,
                                              3 + 2 * k:3 + 2 * k + 2], 'g')

    plt.draw()
    #plt.waitforbuttonpress(0)

#  G_draw_traj_and_map(global_X, global_last_X, global_P, agent_l[0].t)
def G_draw_traj_and_map(X, last_X, P, t):
    """Draw Trajectory and map

    :X: Current state
    :last_X: Previous state
    :P: Covariance
    :t: timestep
    :returns: None

    """
    plt.ion()
    draw_cov_ellipse(X[0:2], P[0:2, 0:2], 'g')
    plt.plot([last_X[0], X[0]], [last_X[1], X[1]], c='k', linewidth=0.75)
    # plt.plot([last_X[0], 0], [last_X[1], 1], c='k', linewidth=0.75)
    plt.plot(X[0], X[1], '*k')

    if t == 0:
        for k in range(6):
            draw_cov_ellipse(
                X[3 + k * 2:3 + k * 2 + 2], P[3 + k * 2:3 + 2 * k + 2,
                                              3 + 2 * k:3 + 2 * k + 2], 'y')
    else:
        for k in range(6):
            draw_cov_ellipse(
                X[3 + k * 2:3 + k * 2 + 2], P[3 + 2 * k:3 + 2 * k + 2,
                                              3 + 2 * k:3 + 2 * k + 2], 'y')

    plt.draw()
    #plt.waitforbuttonpress(0)

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

def init_landmarks2(init_measure, init_measure_cov, init_pose, init_pose_cov):
    '''
    TODO: initialize landmarks given the initial poses and measurements with their covariances
    \param init_measure Initial measurements in the form of (beta0, l0, beta1, l1, ...).
    \param init_measure_cov Initial covariance matrix of shape (2, 2) per landmark given parameters.
    \param init_pose Initial pose vector of shape (3, 1).
    \param init_pose_cov Initial pose covariance of shape (3, 3) given parameters.

    \return k Number of landmarks.
    \return landmarks Numpy array of shape (2k, 1) for the state.
    \return landmarks_cov Numpy array of shape (2k, 2k) for the uncertainty.
    '''

    k = init_measure.shape[0] // 2

    landmark = np.zeros((2 * k, 1))
    landmark_cov = np.zeros((2 * k, 2 * k))

    init_pose = init_pose.reshape(-1) #change from (3,1) to (3,)
    init_measure = init_measure.reshape(-1)
    bot_x = init_pose[0]
    bot_y = init_pose[1]
    bot_theta = init_pose[2]
    for i in range(k):
        beta = init_measure[i*2]
        l = init_measure[i*2 + 1]
        x = bot_x + np.cos(bot_theta + beta) * l
        y = bot_y + np.sin(bot_theta + beta) * l
        
        landmark[i * 2] = x
        landmark[i*2 + 1] = y

        trans = np.asarray([[-1 *l * np.sin(bot_theta + beta), np.cos(bot_theta + beta)], [l * np.cos(bot_theta + beta), np.sin(bot_theta + beta)]])
        landmark_cov[i*2:i*2+2, i*2:i*2+2] = trans @ init_measure_cov @ trans.T + np.diag(init_pose_cov[0:2])


    return k, landmark, landmark_cov
def init_landmarks(init_measure, init_measure_cov, init_pose, init_pose_cov):
    '''
    TODO: initialize landmarks given the initial poses and measurements with their covariances
    \param init_measure Initial measurements in the form of (beta0, l0, beta1, l1, ...).
    \param init_measure_cov Initial covariance matrix of shape (2, 2) per landmark given parameters.
    \param init_pose Initial pose vector of shape (3, 1).
    \param init_pose_cov Initial pose covariance of shape (3, 3) given parameters.

    \return k Number of landmarks.
    \return landmarks Numpy array of shape (2k, 1) for the state.
    \return landmarks_cov Numpy array of shape (2k, 2k) for the uncertainty.
    '''
    # number of landmarks
    k = init_measure.shape[0] // 2
    # initial pose x y theta
    x, y, theta = init_pose.ravel()

    # initialize landmarks arrays and covariance
    landmark = np.zeros((2 * k, 1))
    landmark_cov = np.zeros((2 * k, 2 * k))

    # loop calculation for landmark position and covariance
    for i in range(k):
        # measurements beta (bearing angle) and r (range)
        beta = warp2pi(init_measure[2 * i, 0])
        r = init_measure[2 * i + 1, 0]
        
        # calculate landmark position
        lx = x + r * np.cos(theta + beta) 
        ly = y + r * np.sin(theta + beta)
        landmark[2 * i] = lx
        landmark[2 * i + 1] = ly

        # motion model with respect to the state
        Lp = np.array([[-r * np.sin(theta + beta), np.cos(theta + beta)],
                      [r * np.cos(theta + beta), np.sin(theta + beta)]])
        # measurement model with respect to the state
        Ll = np.array([[1, 0, -r * np.sin(theta + beta)],
                       [0, 1, r * np.cos(theta + beta)]])
    
        # update landmark covariance
        landmark_cov[2 * i:2 * i + 2, 2 * i:2 * i + 2] = Lp @ init_measure_cov @ Lp.T + Ll @ init_pose_cov @ Ll.T

    return k, landmark, landmark_cov
def predict(X, P, control, control_cov, k):
    '''
    TODO: predict step in EKF SLAM with derived Jacobians.
    \param X State vector of shape (3 + 2k, 1) stacking pose and landmarks.
    \param P Covariance matrix of shape (3 + 2k, 3 + 2k) for X.
    \param control Control signal of shape (2, 1) in the polar space that moves the robot.
    \param control_cov Control covariance of shape (3, 3) in the (x, y, theta) space given the parameters.
    \param k Number of landmarks.

    \return X_pre Predicted X state of shape (3 + 2k, 1).
    \return P_pre Predicted P covariance of shape (3 + 2k, 3 + 2k).
    '''
    # extract initial pose x, y, theta
    x, y, theta = X[:3].ravel()

    # extract control inputs delta and alpha
    delta, alpha = control.ravel()

    # initialize state vector X and Convariance matrix P
    X_pre = np.zeros((3 + 2 * k, 1))
    P_pre = np.zeros((3 + 2 * k, 3 + 2 * k))

    # predicted next pose
    x_pre = x + delta * np.cos(theta)
    y_pre = y + delta * np.sin(theta)
    theta_pre = warp2pi(theta + alpha)

    # Jacobian of the motion model
    Gt = np.eye (3 + 2 * k)
    Gt[:3, :3] = np.array([[1, 0, -delta * np.sin(theta)],
                  [0, 1, delta * np.cos(theta)],
                  [0, 0, 1]])

    # Jacobian of the control model (rotaion matrix)
    At = np.zeros((3 + 2 * k, 3 + 2 * k))
    At[:3, :3] = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta), np.cos(theta), 0],
                  [0, 0, 1]])
    
    # predict state vector X
    X_pre = np.vstack([x_pre, y_pre, theta_pre, X[3:]])
    
    # expand control_cov matrix to match size (15 by 15)
    control_cov_expand = np.zeros((3 + 2 * k, 3 + 2 * k))
    control_cov_expand[:3, :3] = control_cov

    # predict covariance P
    P_pre = Gt @ P @ Gt.T + At @ control_cov_expand @ At.T
    
    return X_pre, P_pre
def predict2(X, P, control, control_cov, k):
    '''
    TODO: predict step in EKF SLAM with derived Jacobians.
    \param X State vector of shape (3 + 2k, 1) stacking pose and landmarks.
    \param P Covariance matrix of shape (3 + 2k, 3 + 2k) for X.
    \param control Control signal of shape (2, 1) in the polar space that moves the robot.
    \param control_cov Control covariance of shape (3, 3) in the (x, y, theta) space given the parameters.
    \param k Number of landmarks.

    \return X_pre Predicted X state of shape (3 + 2k, 1).
    \return P_pre Predicted P covariance of shape (3 + 2k, 3 + 2k).
    '''
    # extract initial pose x, y, theta
    x, y, theta = X[:3].ravel()

    # extract control inputs delta and alpha
    delta, alpha = control.ravel()

    # initialize state vector X and Covariance matrix P
    X_pre = np.zeros((3 + 2 * k, 1))
    P_pre = np.zeros((3 + 2 * k, 3 + 2 * k))

    # predicted next pose
    x_pre = x + delta * np.cos(theta)
    y_pre = y + delta * np.sin(theta)
    theta_pre = warp2pi(theta + alpha)

    # Jacobian of the motion model
    Gt = np.eye (3 + 2 * k)
    Gt[:3, :3] = np.array([[1, 0, -delta * np.sin(theta)],
                  [0, 1, delta * np.cos(theta)],
                  [0, 0, 1]])

    # Jacobian of the control model (rotaion matrix)
    At = np.zeros((3 + 2 * k, 3 + 2 * k))
    At[:3, :3] = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta), np.cos(theta), 0],
                  [0, 0, 1]])
    
    # predict state vector X
    X_pre = np.vstack([x_pre, y_pre, theta_pre, X[3:]])
    
    # expand control_cov matrix to match size (15 by 15)
    control_cov_expand = np.zeros((3 + 2 * k, 3 + 2 * k))
    control_cov_expand[:3, :3] = control_cov

    # predict covariance P
    P_pre = Gt @ P @ Gt.T + At @ control_cov_expand @ At.T
    
    return X_pre, P_pre
def update(X_pre, P_pre, measure, measure_cov, k):
    '''
    TODO: update step in EKF SLAM with derived Jacobians.
    \param X_pre Predicted state vector of shape (3 + 2k, 1) from the predict step.
    \param P_pre Predicted covariance matrix of shape (3 + 2k, 3 + 2k) from the predict step.
    \param measure Measurement signal of shape (2k, 1).
    \param measure_cov Measurement covariance of shape (2, 2) per landmark given the parameters.
    \param k Number of landmarks.

    \return X Updated X state of shape (3 + 2k, 1).
    \return P Updated P covariance of shape (3 + 2k, 3 + 2k).
    '''
    # extract predicted pose x, y, theta
    x, y, theta = X_pre[:3].ravel()

    # initialize state vector X and Convariance matrix P
    X = X_pre.copy()
    P = P_pre.copy()

    # intialize measurement Jacobian Ht and Hl
    Ht = np.zeros((2 * k, 3 + 2 * k))   # (12 by 15)
    diff = np.zeros((2 * k, 1))         # (12 by 1)

    # loop calculation for each landmark
    for i in range(k):
        # landmark position
        lx, ly = X_pre[3 + 2 * i:3 + 2 * i + 2].ravel()

        # calculate the expected landmark measurements
        dx = lx - x
        dy = ly - y
        pow_dxdy = dx**2 + dy**2
        sqr_dxdy = np.sqrt(pow_dxdy)
        
        # calculate the Jacobians
        # with respect to the robot pose 
        Hp = np.array([[dy / pow_dxdy, -dx / pow_dxdy, -1],
                       [-dx / sqr_dxdy, -dy / sqr_dxdy, 0]])
        # with respect to the landmark measurement
        Hl = np.array([[-dy / pow_dxdy, dx / pow_dxdy],
                       [dx / sqr_dxdy, dy / sqr_dxdy]])
        
        # Combine the Jacobians into the Jacobian Ht
        Ht[2 * i:2 * i + 2, 0:3] = Hp
        Ht[2 * i:2 * i + 2, 3 + 2 * i:3 + 2 * i + 2] = Hl

        # calculate the expected landmark measurements
        diff = measure - _h(X_pre, k)
    
    # measurement noise covariance matrix Qt (12 by 12)
    Qt = np.kron(np.eye(k), measure_cov)
  
    # calculate the Kalman gain (15 by 12)
    Kt = P_pre @ Ht.T @ np.linalg.inv(Ht @ P_pre @ Ht.T + Qt)

    # update the state vector X
    X = X_pre + Kt @ diff

    # update the covariance matrix P
    P = (np.eye(3 + 2 * k) - Kt @ Ht) @ P_pre

    return X, P
 


def evaluate(X, P, k):
    '''
    TODO: evaluate the performance of EKF SLAM.
    1) Plot the results.
    2) Compute and print the Euclidean and Mahalanobis distance given X, P, and the ground truth (provided in the function).
    \param X State vector of shape (3 + 2k, 1) stacking pose and landmarks.
    \param P Covariance matrix of shape (3 + 2k, 3 + 2k) for X.

    \return None
    '''
    l_true = np.array([3, 6, 3, 12, 7, 8, 7, 14, 11, 6, 11, 12], dtype=float)
    plt.scatter(l_true[0::2], l_true[1::2], c = "y")
    plt.draw()
    while not plt.waitforbuttonpress(): pass

    err = l_true - X[3:].reshape(-1)
    euclidean = []
    mahalanobis = []
    for i in range(k):
        xerr, yerr = err[i*2:i*2+2]
        euclidean.append(np.linalg.norm([xerr, yerr]))
        print("For ", i, "th waypoint, ")
        print("Euclidean error is", euclidean[-1])
        errs = np.array([xerr, yerr])
        cov = P[3+i*2:3+i*2+2, 3+i*2:3+i*2+2]
        mahalanobis.append(math.sqrt(errs @ np.linalg.inv(cov) @ errs.T))
        print("mahalanobis error is", mahalanobis[-1])
        continue

class Agent():
    def __init__(self, file_name, control_cov, measure_cov):
        #initialize for every agent base on first measurements
        self.data = open(file_name).readlines()
        self.control_cov = control_cov
        self.measure_cov = measure_cov
        pose = np.zeros((3, 1))
        pose_cov = np.diag([0.02**2, 0.02**2, 0.1**2])
        line = self.data[0]
        fields = re.split('[\t ]', line)[:-1]
        arr = np.array([float(field) for field in fields])
        measure = np.expand_dims(arr, axis=1)
        k, landmark, landmark_cov = init_landmarks(measure, measure_cov, pose,
                                               pose_cov)
        self.X = np.vstack((pose, landmark))
        self.P = np.block([[pose_cov, np.zeros((3, 2 * k))],
                    [np.zeros((2 * k, 3)), landmark_cov]])
        self.k = k
        self.t = 1
        self.last_X = self.X
        self.X_pre = np.zeros([3+2*k, 1])
        self.P_pre = np.zeros([3+2 * k, 3 + 2 * k])
        
    #Use original predict and update functions
    # def predict():
    #     return;
    # def update():
    #     return;
    def step(self, i):
        line = self.data[i]
        fields = re.split('[\t ]', line)[:-1]
        arr = np.array([float(field) for field in fields])
        # Control
        if arr.shape[0] == 2:
            print(f'{self.t}: Predict step')
            d, alpha = arr[0], arr[1]
            control = np.array([[d], [alpha]])

            ##########
            # TODO: predict step in EKF SLAM
            self.X_pre, self.P_pre = predict(self.X, self.P, control, self.control_cov, self.k)

            draw_traj_and_pred(self.X_pre, self.P_pre)

        # Measurement
        else:
            print(f'{self.t}: Update step')
            measure = np.expand_dims(arr, axis=1)

            ##########
            # TODO: update step in EKF SLAM
            self.X, self.P = update(self.X_pre, self.P_pre, measure, self.measure_cov, self.k)

            draw_traj_and_map(self.X, self.last_X, self.P, self.t)
            self.last_X = self.X
            self.t += 1
        # self.X = X
        # self.P = P
        


    

def multi_main():
    # Setup: same as original EKF

    # TEST: Setup uncertainty parameters
    sig_x = 0.25
    sig_y = 0.1
    sig_alpha = 0.1
    sig_beta = 0.01
    sig_r = 0.08

    # sig_x = 2.5
    # sig_y *= 10
    # sig_alpha *= 10
    # sig_beta *= 10
    # sig_r *= 10


    # Generate variance from standard deviation
    sig_x2 = sig_x**2
    sig_y2 = sig_y**2
    sig_alpha2 = sig_alpha**2
    sig_beta2 = sig_beta**2
    sig_r2 = sig_r**2

    # Open data file and read the initial measurements
    data_file = open("data/data.txt")
    line = data_file.readline()
    fields = re.split('[\t ]', line)[:-1]
    arr = np.array([float(field) for field in fields])
    measure = np.expand_dims(arr, axis=1)
    t = 1

    # Setup control and measurement covariance
    control_cov = np.diag([sig_x2, sig_y2, sig_alpha2])
    measure_cov = np.diag([sig_beta2, sig_r2])

    # Setup the initial pose vector and pose uncertainty
    pose = np.zeros((3, 1))
    pose_cov = np.diag([0.02**2, 0.02**2, 0.1**2])

    num_agent = 2
    agent_l = []

    
    # Initialize every agent
    a0 = Agent("data/data.txt", control_cov, measure_cov)
    a1 = Agent("data/data_bs.txt", control_cov, measure_cov)
    agent_l.append(a0)
    agent_l.append(a1)

    num_landmarks = int(a1.P.shape[0] - 3) 


    # In for loop, call predict and update for every agent.
    # We also need to make sure to update the landmark portions of the state vectors for each agent.
    # This will occur after the update step.
    for i in range(1, len(data_file.readlines())):
        for agent in agent_l:
            agent.step(i)

            # Now all agents have done their predict and update steps, so we want to modify the landmark vectors. 
            global_X = np.zeros_like(agent_l[0].X)
            # global_P = np.zeros_like(agent_l[0].P)

            # We are technically breaking this down by x and y landmarks.
            weights = np.zeros((num_landmarks, num_agent))

            '''Compute a weighted average based on the covariances. 
            We will use this weighted average to come up with a new agent.X for each agent. 
            Each agent has their own covariance matrix. We want to compute a weight for each agent
            for each landmark.'''

            for landmark_ind in range(num_landmarks):
                for agent_ind in range(num_agent):
                    cov_agent = agent_l[agent_ind].P
                    norm = np.linalg.norm(cov_agent[3 + landmark_ind, :])
                    
                    weights[landmark_ind, agent_ind] = norm
                

            # We need to normalize the weights for each landmark. 
            row_sums = np.sum(weights, axis=1, keepdims=True)
            print(f'row sums has shape {row_sums.shape}')
            weights /= row_sums
            
            # We will now combine the state vectors into one state vector.
            for landmark_ind in range(num_landmarks):
                # Take the weighted average of the X matrix using all agents.
                total = 0.0
                for agent_ind in range(num_agent):
                    weight = weights[landmark_ind, agent_ind]
                    landmark_value = agent_l[agent_ind].X[3 + landmark_ind]

                    total += weight * landmark_value

                    # Weights should be normalized so no need to divide by the sum of weights. 
                global_X[landmark_ind] = total 

            
            # Finally, set all agents' landmark values to the global X. 
            for agent_ind in range(num_agent):
                agent_i = agent_l[agent_ind]
                agent_i_curr_pose = agent_i.X[:3]

                agent_i.X = global_X
                agent_i.X[:3] = agent_i_curr_pose

    




        
    

    






    #global_last_X = a0.X
    # In for loop, call predict and update for every agent5
    '''for i in range(1,len(data_file.readlines())):
        for agent in agent_l:
            agent.step(i)
        global_X = np.zeros_like(agent_l[0].X)
        global_P = np.zeros_like(agent_l[0].P)
        for agent in agent_l:
            global_X += agent.X
            global_P += agent.P
        global_X = global_X / len(agent_l)
        global_P = global_P / len(agent_l)
        G_draw_traj_and_map(global_X, global_last_X, global_P, agent_l[0].t)
        global_last_X = global_X'''
    print("\n Evaluating Agent 0")
    evaluate(a0.X, a0.P, a0.k)
    print("\n Evaluating Agent 1")
    evaluate(a1.X, a1.P, a1.k)
    #print("\n Evaluating Global")   
    #evaluate(global_X, global_P, a0.k)       



    return


if __name__ == "__main__":
    multi_main()
