
import joblib
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

matfile = scipy.io.loadmat('/home/horizon/Desktop/cloning_behavior_mat/general_mode_concat_driver123.mat')
model_label = matfile['class'][:]
# mode_label = mode_label.transpose()
# mode_label.shape
#print(mode_label)
Y = model_label.reshape((14851,))
X= matfile['phi'][:,:]


test = np.array(X)[0:30,:]
label = np.array(Y)[0:30]
print(label)
print(test)
loaded_model = joblib.load('svm_params.sav')
flg = loaded_model.predict(test)
print(flg)