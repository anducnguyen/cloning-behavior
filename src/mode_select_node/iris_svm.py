import scipy.io 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from sklearn import svm, datasets
import scipy.cluster
import sklearn.cluster
import pandas as pd


iris = datasets.load_iris()

print(iris.keys())
#dict_keys(['data', 'target', 'target_names', 'DESCR', 'feature_names', 'filename'])

# Take the first two features. We could avoid this by using a two-dim dataset
X = iris.data[:,:4]
Y = iris.target
# tab = pd.DataFrame(iris.data, columns = iris.feature_names)
# print(tab)
C = 1.0
print(X)
model = svm.SVC(kernel = 'linear', C=C)
#model = svm.SVC(kernel='rbf', gamma=0.7, C=C)
clf = model.fit(X, Y)
print(Y)
z1 = lambda x1,y1: (-clf.intercept_[0]-clf.coef_[0][0]*x -clf.coef_[0][1]*y) / clf.coef_[0][2]
z2 = lambda x2,y2: (-clf.intercept_[1]-clf.coef_[1][0]*x -clf.coef_[1][1]*y) / clf.coef_[1][2]
z3 = lambda x3,y3: (-clf.intercept_[1]-clf.coef_[2][0]*x -clf.coef_[2][1]*y) / clf.coef_[2][2]
tmp = np.linspace(0,10,50)
x,y = np.meshgrid(tmp,tmp)



fig = plt.figure()

ax = fig.gca(projection='3d')
figure = ax.scatter(X[Y==0,0], X[Y==0,1], X[Y==0,2],c="b")
figure = ax.scatter(X[Y==1,0], X[Y==1,1], X[Y==1,2],c="r")
figure = ax.scatter(X[Y==2,0], X[Y==2,1], X[Y==2,2],c="g")
ax.set_xlabel('X1')
ax.set_ylabel('X2')
ax.set_zlabel('X3')
ax.plot_surface(x, y, z1(x,y))
ax.plot_surface(x, y, z2(x,y))
#ax.plot_surface(x, y, z3(x,y))
ax.view_init(30, 60)
plt.show()