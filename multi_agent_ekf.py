'''
    Initially written by Ming Hsiao in MATLAB
    Adapted to Python by Akash Sharma (akashsharma@cmu.edu), 2020
    Updated by Wei Dong (weidong@andrew.cmu.edu), 2021
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
    #draw_cov_ellipse(X[0:2], P[0:2, 0:2], 'm')
    #plt.draw()
    return


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

def G_draw_traj_and_map(X, last_X, P, t):
    """Draw Trajectory and map

    :X: Current state
    :last_X: Previous state
    :P: Covariance
    :t: timestep
    :returns: None

    """
    plt.ion()
    # draw_cov_ellipse(X[0:2], P[0:2, 0:2], 'g')
    # plt.plot([last_X[0], X[0]], [last_X[1], X[1]], c='k', linewidth=0.75)
    # plt.plot(X[0], X[1], '*k')

    if t <26:
        # for k in range(6):
        #     draw_cov_ellipse(
        #         X[3 + k * 2:3 + k * 2 + 2], P[3 + k * 2:3 + 2 * k + 2,
        #                                       3 + 2 * k:3 + 2 * k + 2], 'r')
        pass
    else:
        for k in range(6):
            draw_cov_ellipse(
                X[3 + k * 2:3 + k * 2 + 2], P[3 + 2 * k:3 + 2 * k + 2,
                                              3 + 2 * k:3 + 2 * k + 2], 'r')

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
        # print("For ", i, "th waypoint, ")
        # print("Euclidean error is", euclidean[-1])
        errs = np.array([xerr, yerr])
        cov = P[3+i*2:3+i*2+2, 3+i*2:3+i*2+2]
        mahalanobis.append(math.sqrt(errs @ np.linalg.inv(cov) @ errs.T))
        # print("mahalanobis error is", mahalanobis[-1])
        continue
    print(euclidean)
    print(mahalanobis)
    return

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

            #draw_traj_and_pred(self.X_pre, self.P_pre)

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

def aggregate_covariance_matrices(agent_list):
    all_information_matrices = [np.linalg.inv(agent.P) for agent in agent_list]
    global_information_matrix = np.sum(all_information_matrices, axis=0)
    global_covariance = np.linalg.inv(global_information_matrix)
    return global_covariance


def analyze_global_covariance(global_covariance):
    # Analyze the global covariance matrix
    trace = np.trace(global_covariance)
    eigenvalues, _ = np.linalg.eig(global_covariance)
    # Perform further analysis as needed
    return trace, eigenvalues


def print_agent_variances(agent_list):
    for i, agent in enumerate(agent_list):
        variance = np.trace(agent.P)
        print(f"Agent {i} variance (trace of covariance matrix): {variance}")
        variance = np.linalg.det(agent.P)
        print(f"Agent {i} variance (determinant of covariance matrix): {variance}")        

def compute_weighted_avg(agent_l, num_landmarks):
    num_agent = len(agent_l)
    weights = np.zeros((num_landmarks, num_agent))
    global_X = np.zeros_like(agent_l[0].X)

    for landmark_ind in range(num_landmarks):
        for agent_ind in range(num_agent):
            cov_agent = agent_l[agent_ind].P
            norm = np.linalg.norm(cov_agent[3 + landmark_ind * 2 : 3 + landmark_ind * 2 + 2, :])
            weights[landmark_ind, agent_ind] = norm
    row_sums = np.sum(weights, axis=1)
    weights = (weights.T / row_sums).T
    for landmark_ind in range(num_landmarks):
        # Take the weighted average of the X matrix using all agents.
        total = 0.0
        for agent_ind in range(num_agent):
            weight = weights[landmark_ind, agent_ind]
            landmark_value = agent_l[agent_ind].X[3 + landmark_ind * 2: 5 + landmark_ind * 2]

            total += weight * landmark_value


            # Weights should be normalized so no need to divide by the sum of weights. 
            global_X[3 + 2 * landmark_ind:3 + 2 * landmark_ind +2] = total
    return global_X

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
    measure_cov0 = np.diag([0.01**2, 0.09**2])
    measure_cov1 = np.diag([0.009**2, 0.08**2])
    measure_cov2 = np.diag([0.008**2, 0.07**2])

    # Setup the initial pose vector and pose uncertainty
    pose = np.zeros((3, 1))
    pose_cov = np.diag([0.02**2, 0.02**2, 0.1**2])

    num_agent = 2
    agent_l = []

    
    # Initialize every agent
    a0 = Agent("dataP0R5B4.txt", control_cov, measure_cov0)
    a1 = Agent("dataP1R4B3.txt", control_cov, measure_cov1)
    a2 = Agent("dataP2R3B2.txt", control_cov, measure_cov2)
    agent_l.append(a0)
    agent_l.append(a1)
    agent_l.append(a2)
    global_last_X = a0.X
    num_landmarks = int((agent_l[0].P.shape[0] - 3)/2) 
    # In for loop, call predict and update for every agent5
    for i in range(1,len(data_file.readlines())):
        counter = 0
        for agent in agent_l:#step each agent
            print(counter)
            counter += 1
            agent.step(i)
        global_X = np.zeros_like(agent_l[0].X)
        global_P = np.zeros_like(agent_l[0].P)
        
        for agent in agent_l:#take simple average as global estimation 
            global_X += agent.X
            global_P += agent.P
        global_X = global_X / len(agent_l)
        global_P = global_P / len(agent_l)
        

        global_X = compute_weighted_avg(agent_l, num_landmarks)
        G_draw_traj_and_map(global_X, global_last_X, global_P, agent_l[0].t) 
        
        global_last_X = global_X


    global_covariance = aggregate_covariance_matrices(agent_l)
    trace, eigenvalues = analyze_global_covariance(global_covariance)
    global_det = np.linalg.det(global_covariance)
    

    print("global variance (trace of covariance matrix):", trace)
    print("Global variance (determinant of covariance matrix): ",global_det)
    for i in range(len(agent_l)):
        c = aggregate_covariance_matrices([agent_l[i]])
        trace, eigenvalues = analyze_global_covariance(c)
        global_det = np.linalg.det(c)
        print("agent ", i, "  variance (trace of covariance matrix):", trace)
        print("agent ", i, " (determinant of covariance matrix): ",global_det)


    print("\n Evalutating Agent 0")
    evaluate(a0.X, a0.P, a0.k)
    print("\n Evalutating Agent 1")
    evaluate(a1.X, a1.P, a1.k)
    print("\n Evalutating Agent 2")
    evaluate(a2.X, a2.P, a2.k)
    print("\n Evalutating Global")   
    evaluate(global_X, global_P, a0.k)       



    return

def main():
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

    ##########
    # TODO: initialize landmarks
    k, landmark, landmark_cov = init_landmarks(measure, measure_cov, pose,
                                               pose_cov)

    # Setup state vector X by stacking pose and landmark states
    # Setup covariance matrix P by expanding pose and landmark covariances
    X = np.vstack((pose, landmark))
    P = np.block([[pose_cov, np.zeros((3, 2 * k))],
                  [np.zeros((2 * k, 3)), landmark_cov]])

    # Plot initial state and covariance
    last_X = X
    draw_traj_and_map(X, last_X, P, 0)

    # Core loop: sequentially process controls and measurements
    for line in data_file:
        fields = re.split('[\t ]', line)[:-1]
        arr = np.array([float(field) for field in fields])

        # Control
        if arr.shape[0] == 2:
            print(f'{t}: Predict step')
            d, alpha = arr[0], arr[1]
            control = np.array([[d], [alpha]])

            ##########
            # TODO: predict step in EKF SLAM
            X_pre, P_pre = predict(X, P, control, control_cov, k)

            draw_traj_and_pred(X_pre, P_pre)

        # Measurement
        else:
            print(f'{t}: Update step')
            measure = np.expand_dims(arr, axis=1)

            ##########
            # TODO: update step in EKF SLAM
            X, P = update(X_pre, P_pre, measure, measure_cov, k)

            draw_traj_and_map(X, last_X, P, t)
            last_X = X
            t += 1

    # EVAL: Plot ground truth landmarks and analyze distances
    evaluate(X, P, k)


if __name__ == "__main__":
    multi_main()
