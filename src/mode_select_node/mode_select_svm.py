import scipy.io 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from sklearn import svm, datasets
import scipy.cluster
import sklearn.cluster
import pandas as pd
import joblib

# columns =['data', 'target', 'target_names', 'DESCR', 'feature_names', 'filename']
# df_train = pd.DataFrame(iris_data, columns=columns)

matfile = scipy.io.loadmat('/home/arun/Desktop/an_ws/cloning_behavior_mat/general_mode_concat_driver123.mat')
model_label = matfile['class'][:]
# mode_label = mode_label.transpose()
# mode_label.shape
#print(mode_label)
Y = model_label.reshape((14851,))
X= matfile['phi'][:,:]
#input_matrix = input_matrix.transpose()
# input_matrix.shape

#print(input_matrix)
#print("This is mode_class array", mode_class)
# arrdata = np.array(data)
#print(data.keys())
#print("this is data matrix size:",len(data['driver1_data1'].transpose()))
# ego_speed = data['phi'][0:-1,5]
# ego_acc = data['driver1_data1'][0:-1,6]
# range_ef = data['driver1_data1'][0:-1,7]
# range_rate = data['driver1_data1'][0:-1,7]

clf = svm.SVC(kernel = 'linear',decision_function_shape='ovr')
clf.fit(X, Y)

test = np.array(X)[0:30,:]
label = np.array(Y)[0:30]
print(label)
print(test)
flg = clf.predict(test)
print("Mode Selection in Real Time")
print(flg)
print("SVM coeff")
print(clf.coef_)
# z1 = lambda x1,y1: (-clf.intercept_[0]-clf.coef_[0][0]*x -clf.coef_[0][1]*y) / clf.coef_[0][2]
# z2 = lambda x2,y2: (-clf.intercept_[1]-clf.coef_[1][0]*x -clf.coef_[1][1]*y) / clf.coef_[1][2]
# z3 = lambda x3,y3: (-clf.intercept_[1]-clf.coef_[2][0]*x -clf.coef_[2][1]*y) / clf.coef_[2][2]
# tmp = np.linspace(-2,2,5)
# x,y = np.meshgrid(tmp,tmp)



# fig = plt.figure()

# ax = fig.gca(projection='3d')
# figure = ax.scatter(X[Y==1,4], X[Y==1,5], X[Y==1,3],s=1,c="r")
# figure = ax.scatter(X[Y==2,4], X[Y==2,5], X[Y==2,3],s=1,c="g")
# figure = ax.scatter(X[Y==3,4], X[Y==3,5], X[Y==3,3],s=1,c="b")
# figure = ax.scatter(X[Y==4,4], X[Y==4,5], X[Y==4,3],s=1,c="y")
# figure = ax.scatter(X[Y==5,4], X[Y==5,5], X[Y==5,3],s=1,c="m")
# ax.set_xlabel('X1')
# ax.set_ylabel('X2')
# ax.set_zlabel('X3')
# #ax.plot_surface(x, y, z1(x,y))
# #ax.plot_surface(x, y, z2(x,y))
# #ax.plot_surface(x, y, z3(x,y))
# ax.view_init(90, 0)
# plt.show()


filename = 'svm_params.sav'
joblib.dump(clf, filename)
# print("this is speed data:", speed)
# fig = plt.figure()
# ax = fig.gca(projection='3d')
# figure = ax.scatter(speed, acc, r,c=acc, cmap='viridis', linewidth=0.05)
# ax.set_xlabel('speed')
# ax.set_ylabel('acc')input_matrix
# plt.show()
# A = np.vstack((speed, acc,r)).T
# print("this is A matrix",A)

# kmeans = sklearn.cluster.KMeans(12, max_iter=300)
# kmeans.fit(A)
# means = kmeans.cluster_centers_
# labels = kmeans.labels_
# #plt.scatter(A[:, 0], A[:, 1],c=labels)
# fig = plt.figure()
# ax = fig.gca(projection='3d')
# figure = ax.scatter(A[:, 0],A[:, 1], A[:, 2],c=labels, cmap='viridis', linewidth=0.05)
# ax.set_xlabel('speed')
# ax.set_ylabel('acc')
# ax.set_zlabel('range');
# #plt.scatter(means[:, 0], means[:, 1], linewidths=2)

# plt.show()


# X = [[0, 0], [10, 1]]
# y = [0, 1]
# clf = svm.SVC()
# clf.fit(X, y)
# flg = clf.predict([[2., 2.],[1., 1.],[310., 2.5]])
# print(flg)
# params  = np.array([1, 1, 3])
# flg_output = np.dot(flg,params.transpose())
# print(flg_output)


