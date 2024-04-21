##For mahalanobis and Euclidean error
import numpy as np
import matplotlib.pyplot as plt

# Data for evaluation
if False:
    agents = ['Agent 0', 'Agent 1', 'Agent 2', 'Global']
    waypoints = np.arange(6)
    euclidean_errors = np.array([
        [0.0495441103651212, 0.09815331620517925, 0.12853450630590768, 0.13826911758653238, 0.09738601737325121, 0.10093571699471775],
        [0.12661377464683843, 0.3941343640250028, 0.33364386474021773, 0.4856458775174728, 0.4395794099497029, 0.5166167788097684],
        [0.16505081203425603, 0.25050566895870807, 0.1246223243082488, 0.27880639664334605, 0.2799923792123846, 0.22557596450430895],
        [0.032782603552493815, 0.03547575181206919, 0.024111799297660438, 0.0059196695596440045, 0.005001186423020551, 0.04501473949745334]
    ])
    mahalanobis_errors = np.array([
        [1.0875375464470272, 0.3038698644256682, 1.2621050270838488, 0.8604557103381542, 0.6121852859918082, 0.34090098757087095],
        [1.6220320816961105, 1.2828608072653906, 1.8346904440771028, 1.143625869059169, 1.2515586085902797, 1.1424674149820566],
        [1.2945596660529952, 1.576882978614281, 0.39212768717916635, 1.4511600289226285, 0.7673339260328471, 0.4511774818201813],
        [0.5571258100776164, 0.6756448430635583, 0.1315614350983756, 0.062486559982498656, 0.08239253122356079, 0.09737105555751659]
    ])

    # Plotting
    fig, ax = plt.subplots(figsize=(10, 6))
    bar_width = 0.2
    opacity = 0.8
    colors = ['b', 'g', 'r', 'y']

    for i in range(len(agents)):
        plt.bar(waypoints + i * bar_width, mahalanobis_errors[i], bar_width,
                alpha=opacity,
                color=colors[i],
                label=agents[i] + ' (Mahalanobis)')
        plt.bar(waypoints + i * bar_width, euclidean_errors[i], bar_width,
                alpha=opacity,
                color=colors[i],
                label=agents[i] + ' (Euclidean)'
                ,hatch='//')
        
        

    plt.xlabel('Waypoints')
    plt.ylabel('Error')
    plt.title('Evaluation of Agents and Global Metrics')
    plt.xticks(waypoints + bar_width * 1.5, waypoints)
    plt.legend()

    plt.tight_layout()
    plt.show()


##For trace and determinant
if True:

# Data for variance
    agents = ['Agent 0', 'Agent 1', 'Agent 2', 'Global']
    trace_variances = [0.753, 0.953, 1.191, 0.0489]
    determinant_variances = [1.605731002205155e-47, 8.128877947772323e-47, 6.0390719597285912e-47, 1.21073489969104e-55]

    # Plotting
    fig, ax = plt.subplots(figsize=(10, 6))

    bar_width = 0.35
    opacity = 0.8
    colors = ['b', 'g', 'r', 'y']

    # Plotting trace variances
    ax.bar(np.arange(len(agents)), trace_variances, bar_width,
            alpha=opacity,
            color=colors,
            label='Trace of Covariance Matrix')

    # Create a secondary y-axis for determinant variances
    ax2 = ax.twinx()

    # Plotting determinant variances with logarithmic scale
    ax2.bar(np.arange(len(agents)) + bar_width, determinant_variances, bar_width,
            alpha=opacity,
            color=colors,
            label='Determinant of Covariance Matrix',
            hatch='//')

    ax.set_xlabel('Entities')
    ax.set_ylabel('Trace of Covariance Matrix')
    ax2.set_ylabel('Log(Determinant of Covariance Matrix)')

    # Set x-ticks and labels
    ax.set_xticks(np.arange(len(agents)) + bar_width / 2)
    ax.set_xticklabels(agents)

    # Use logarithmic scale for y-axis
    ax2.set_yscale('log')

    # Display legends
    ax.legend(loc='upper left')
    ax2.legend(loc='upper right')

    plt.title('Variance of Agents and Global Metrics')

    plt.show()

