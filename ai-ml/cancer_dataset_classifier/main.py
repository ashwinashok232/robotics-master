#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr  2 00:28:19 2021

@author: ashwinashok
"""

#Importing all relevant libraries

import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
import sklearn
import matplotlib.pyplot as plt
from sklearn.linear_model import SGDClassifier
from sklearn.metrics import confusion_matrix
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import cross_val_predict
from sklearn.metrics import precision_recall_curve
from sklearn.metrics import accuracy_score
from sklearn.metrics import roc_curve
from sklearn.metrics import roc_auc_score


df = pd.read_csv('wdbc.dataset.csv', header=None) #Importing data as pandas dataframe

#Scaling data to normalize it
df_max = df.max(axis=0)
df_min = df.min(axis=0)
for i in range(2,df.shape[1]):
    df.iloc[:,i] = (df.iloc[:,i]-df_min[i])/(df_max[i]-df_min[i]) #Scaling equation -> (df - min)/(max-min)
    
x = df.iloc[:,2:-1] #Setting x = feature data
y = df.iloc[:,1] #Setting y = traget values

#Converting target data from string to binary (Malignant = 1, Benign = 0) to allow fitting
y = y.replace(to_replace='M',value=1)
y = y.replace(to_replace='B',value=0)

#Splitting data w/ 80% training and 20% test
x_train,x_test,y_train,y_test = train_test_split(x,y,train_size=0.8,random_state=42)

#Running SGD classifier to fit data
sgd_clf = SGDClassifier(max_iter=50, random_state=42) #Random state integr chosen to get reproducible results
sgd_clf.fit(x_train, y_train)
y_test_pred = sgd_clf.predict(x_test) #Testing model on test data to find predicted values

#Confusion Matrix
cm = confusion_matrix(y_test, y_test_pred)
tp = cm[1,1] #True positive
fp = cm[0,1] #False positive
tn = cm[0,0] #True negative
fn = cm[1,0] #False negative
print(cm)

#Accuracy
acc = (tp+tn)/(tp+tn+fp+fn)
print('Accuracy = ',acc)

#Precision
prec = tp/(tp+fp)
print('Precision = ',prec)

#Recall
rec = tp/(tp+fn)
print('Recall = ',rec)

#Predicted value from cross validation
y_pred = cross_val_predict(sgd_clf, x_train, y_train,method="decision_function")

#Cross validation score with cv=5 (default)
score = sklearn.model_selection.cross_val_score(sgd_clf, x_train, y_train,scoring='accuracy')
print(score)
print('Average = ',np.mean(score))
print('Standard Deviation = ',np.std(score))

fpr, tpr, thresholds = roc_curve(y_train, y_pred)

def plot_roc_curve(fpr, tpr, label=None):
    plt.plot(fpr, tpr, linewidth=2, label=label)
    plt.plot([0, 1], [0, 1], 'k--')
    plt.axis([0, 1, 0, 1])
    plt.xlabel('False Positive Rate', fontsize=16)
    plt.ylabel('True Positive Rate', fontsize=16)

plt.figure(figsize=(8, 6))
plot_roc_curve(fpr, tpr)
#save_fig("roc_curve_plot")
plt.show()

print('Area Under the Curve Score = ',roc_auc_score(y_train, y_pred))


