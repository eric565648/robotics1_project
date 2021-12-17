import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy as dp
from numpy.core.fromnumeric import argmax
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import r2_score,mean_squared_error

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
with open('../data/rank.npy','rb') as f:
    rank = np.load(f,allow_pickle=True)

# read param
param=np.genfromtxt('../data/data_param_new.csv',delimiter=',',dtype=float)

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

# read train and test dataset
train_data =np.genfromtxt('../data/train_data.csv',delimiter=',',dtype=float)
test_data =np.genfromtxt('../data/test_data.csv',delimiter=',',dtype=float)

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

score_pred = reg.predict(param_dataset)
arr1inds = score_true.argsort()
arr1inds = np.flip(arr1inds)
sorted_score_true = score_true[arr1inds[::-1]]
sorted_score_pred = score_pred[arr1inds[::-1]]

ptrend = np.poly1d(np.polyfit(range(len(sorted_score_pred)), sorted_score_pred, 3))
score_pred_trend = np.array([])
for i in range(len(sorted_score_pred)):
    score_pred_trend = np.append(score_pred_trend, ptrend(i))

# plt.plot(range(len(score_pred)),sorted_score_true, 'o', color='gray', label='Groundtruth')
# plt.plot(range(len(score_pred)),sorted_score_pred, 'o', color='peachpuff',markersize=3, label='Prediction')
# plt.plot(range(len(score_pred)),score_pred_trend, 'o', color='orange',label='Prediction Trend')

# plt.title("Score Prediction v.s. GroundTruth")
# plt.ylabel("Emotion Score")
# plt.legend()
# plt.show(block=False)

#################3

# Evaluation Result
eval = np.genfromtxt('../data/evaluation_response.csv',delimiter=',',dtype=float)

emotions = ('100 Happy','75 Happy','50 Happy','25 Happy','10 Happy', '0 Happy')
emotions_eval_order = [5,3,0,2,1,4]
emotion_mean = np.array([])
emotion_std = np.array([])
for i in range(len(emotions)):
    emotion_mean = np.append(emotion_mean,np.mean(eval[:,emotions_eval_order[i]]))
    emotion_std = np.append(emotion_std,np.std(eval[:,emotions_eval_order[i]]))

plt.rcdefaults()
fig, ax = plt.subplots()
ax.barh(np.arange(len(emotions)),emotion_mean,xerr=emotion_std,align='center')
ax.set_yticks(np.arange(len(emotions)))
ax.set_yticklabels(emotions)
ax.invert_yaxis()
ax.set_xlabel("Perceived Happy Likely Scale")
ax.set_title("Do you think the robot is happy?")
plt.show(block=True)





