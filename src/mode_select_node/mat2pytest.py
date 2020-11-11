import scipy.io 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import scipy.cluster
import sklearn.cluster
import sklearn.datasets

data = scipy.io.loadmat('../Data_extract/driver1_data1.mat')
arrdata = np.array(data)
print("this is data matrix size:",len(data['driver1_data1'].transpose()))
speed = data['driver1_data1'][0:-1,5]
acc = data['driver1_data1'][0:-1,6]
r = data['driver1_data1'][0:-1,7]
print("this is speed data:", speed)
fig = plt.figure()
ax = fig.gca(projection='3d')
figure = ax.scatter(speed, acc, r,c=acc, cmap='viridis', linewidth=0.05)
ax.set_xlabel('speed')
ax.set_ylabel('acc')
ax.set_zlabel('range');
plt.show()
A = np.vstack((speed, acc,r)).T
print("this is A matrix",A)

kmeans = sklearn.cluster.KMeans(12, max_iter=300)
kmeans.fit(A)
means = kmeans.cluster_centers_
labels = kmeans.labels_
#plt.scatter(A[:, 0], A[:, 1],c=labels)
fig = plt.figure()
ax = fig.gca(projection='3d')
figure = ax.scatter(A[:, 0],A[:, 1], A[:, 2],c=labels, cmap='viridis', linewidth=0.05)
ax.set_xlabel('speed')
ax.set_ylabel('acc')
ax.set_zlabel('range');
#plt.scatter(means[:, 0], means[:, 1], linewidths=2)

plt.show()

