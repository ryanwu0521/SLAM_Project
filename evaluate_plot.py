##For mahalanobis and Euclidean error
import numpy as np
import matplotlib.pyplot as plt

# Data for evaluation
agents = ['Agent 0', 'Agent 1', 'Agent 2', 'Global']
waypoints = np.arange(6)
euclidean_errors = np.array([
    [0.22902207792160903, 0.43730987868891646, 0.4385287879542296, 0.524805721687924, 0.42223194870438263, 0.4918539455523224],
    [0.26275895883382816, 0.48630331456075376, 0.340147484973441, 0.6610833540172003, 0.47550992927000574, 0.6590252395565271],
    [0.16369301585600113, 0.2469829328284951, 0.19630377335679758, 0.2906115222363801, 0.2020336883349722, 0.3242126434565222],
    [0.12103117403767596, 0.22642543490775605, 0.1936537119772257, 0.29837447172795645, 0.23246974363322145, 0.2757971451251542]
])
mahalanobis_errors = np.array([
    [1.3960876508709754, 1.2612552744189474, 1.7837477781422113, 1.2788184802135332, 1.2396417205308106, 1.0867671229713651],
    [1.5942310078527189, 1.5574826991747661, 1.6278065859181952, 1.9825614225268955, 1.4359446069815702, 1.5934880006445675],
    [2.4095870125978727, 0.8461360890591667, 1.1632896440882758, 0.6684919308615619, 0.7998808023655077, 0.9393621769443621],
    [0.9273523509359267, 0.733286575115639, 0.6719625906190246, 0.7526184384698024, 0.6670035856182596, 0.644261531257583]
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
"""
import numpy as np
import matplotlib.pyplot as plt

# Data for variance
agents = ['Agent 0', 'Agent 1', 'Agent 2', 'Global']
trace_variances = [0.8075309702491044, 0.7864354507843525, 0.9814025870426324, 0.05113183949178298]
determinant_variances = [7.605731002205155e-47, 8.128877947772323e-47, 2.0390719597285912e-47, 1.21073489969104e-55]

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

"""
