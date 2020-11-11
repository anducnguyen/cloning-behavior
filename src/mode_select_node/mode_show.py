from tkinter import *
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
window = Tk()

window.title("Mode Switching")

lbl = Label(window, text="Mode 2\nCollision Avoidance", font=("Arial Bold", 15), bg="red", fg="black")
lbl.grid(column=1, row=1,padx=5, pady=5, sticky="nsew")
lbl = Label(window, text="Mode 5\nLong", font=("Arial Bold", 15), bg="green", fg="black")
lbl.grid(column=1, row=0,padx=5, pady=5, sticky="nsew")
lbl = Label(window, text="Mode 3\nShort", font=("Arial Bold", 15), bg="purple", fg="black")
lbl.grid(column=0, row=1,padx=5, pady=5, sticky="nsew")
lbl = Label(window, text="Mode 1\nMid_A", font=("Arial Bold", 15), bg="yellow", fg="black")
lbl.grid(column=2, row=1,padx=5, pady=5, sticky="nsew")
lbl = Label(window, text="Mode 4\nMid_F", font=("Arial Bold", 15), bg="blue", fg="black")
lbl.grid(column=1, row=2,padx=5, pady=5, sticky="nsew")

window.geometry('400x200')

window.mainloop()
# x = 1
# def main():
# if x == 1
# lbl = Label(window, text="Mode 4\nMid_F", font=("Arial Bold", 15), bg="black", fg="white")
# else
# lbl = Label(window, text="Mode 1\nMid_A", font=("Arial Bold", 15), bg="black", fg="white")

# if __name__ == "__main__":
#     main()



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