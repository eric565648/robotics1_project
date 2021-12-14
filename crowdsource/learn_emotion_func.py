# This script use linear regression
# to learn the emotion prediction function

import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import r2_score,mean_squared_error
from copy import deepcopy as dp
import matplotlib.pyplot as plt
import trueskill as tk

class MData:
    def __init__(self,num) -> None:
        self.num = num
        self.comp = np.array([])
        self.rate=tk.Rating()
    def set_comp(self,comp):
        self.comp = np.append(self.comp,comp)
        return True
    def set_rate(self,rate):
        self.rate=rate

# read rank
with open('/media/eric/Transcend/motion_lib/motion_data/rank.npy','rb') as f:
    rank = np.load(f,allow_pickle=True)

# read param
param=np.genfromtxt('/media/eric/Transcend/motion_lib/motion_data/data_param_new.csv',delimiter=',',dtype=float)

# copy the read parameters to new variables
param_dataset = dp(param)
rank_true = dp(rank)

# drop the velocity type (discrete parameter) from now
param_dataset = np.delete(param_dataset,11,1)

# line up the score and sigma from number 1 to 1000
score_true = np.zeros(len(param_dataset))
sigma_true = np.zeros(len(param_dataset))

for r in rank_true:
    score_true[r.num-1] = r.rate.mu
    sigma_true[r.num-1] = r.rate.sigma

# standardize all parameters (training/testing data)
param_dataset_scaler = StandardScaler().fit(param_dataset)
param_dataset_std = param_dataset_scaler.transform(param_dataset)
print(param_dataset[0,:])
print(param_dataset_std[0,:])

# choose 80% for training and 20% for testing
all_num = np.arange(len(param_dataset))
test_data = np.random.choice(all_num,int(len(param_dataset)*0.2),replace=False) # 20% for testing 
test_data = np.sort(test_data)
train_data = np.array([])
for num in all_num:
    if num not in test_data:
        train_data = np.append(train_data,num)
# print("train_data:",train_data)
# print("test_data:",test_data)

# training data and ground truth
param_trainset = np.zeros((len(train_data),len(param_dataset[0,:])))
score_trainset = np.zeros(len(train_data))
sigma_trainset = np.zeros(len(train_data))
for i in range(len(train_data)):
    param_trainset[i] = param_dataset[int(train_data[i])]
    score_trainset[i] = score_true[int(train_data[i])]
    sigma_trainset[i] = sigma_true[int(train_data[i])]
# testing data and ground truth
param_testset = np.zeros((len(test_data),len(param_dataset[0,:])))
score_testset = np.zeros(len(test_data))
sigma_testset = np.zeros(len(test_data))
for i in range(len(test_data)):
    param_testset[i] = param_dataset[int(test_data[i])]
    score_testset[i] = score_true[int(test_data[i])]
    sigma_testset[i] = sigma_true[int(test_data[i])]

# Start fitting with linear regression
reg = LinearRegression()
reg.fit(param_trainset,score_trainset,sample_weight=1./sigma_trainset)

# check coefficient
print('Coefficients: \n', reg.coef_)

# predict and check R2 score
score_pred = reg.predict(param_testset)
print("MSE:",mean_squared_error(score_testset,score_pred))
print("R2:",r2_score(score_testset,score_pred))

# residuals = score_testset-score_pred
# plt.plot(range(len(score_testset)),residuals, 'o', color='darkblue')
# plt.title("Residual Plot")
# plt.xlabel("Independent Variable")
# plt.ylabel("Residual")
# plt.show(block=False)

score_pred = reg.predict(param_dataset)
arr1inds = score_true.argsort()
arr1inds = np.flip(arr1inds)
sorted_score_true = score_true[arr1inds[::-1]]
sorted_score_pred = score_pred[arr1inds[::-1]]
plt.plot(range(len(score_pred)),sorted_score_true, 'o', color='gray')
plt.plot(range(len(score_pred)),sorted_score_pred, 'o', color='orange')
plt.title("Score Prediction v.s. GroundTruth")
plt.ylabel("Score")
plt.show(block=False)

# save standardize coefficient for motion sampling
save_learned_coeff = np.array([param_dataset_scaler.mean_,param_dataset_scaler.scale_,reg.coef_])
np.savetxt('/media/eric/Transcend/motion_lib/motion_data/learn_coeff.csv', save_learned_coeff, delimiter=',')
np.savetxt('/media/eric/Transcend/motion_lib/motion_data/learn_coeff_intercept.csv', np.array([reg.intercept_]),delimiter=',')
np.savetxt('/media/eric/Transcend/motion_lib/motion_data/train_data.csv',train_data,delimiter=',')
np.savetxt('/media/eric/Transcend/motion_lib/motion_data/test_data.csv',test_data,delimiter=',')