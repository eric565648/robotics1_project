import numpy as np
import math
from copy import deepcopy as dp
import trueskill as tk
import cv2
import pygame as pg

# toy_set = np.arange(1000)

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

# show rank
with open('../data/rank.npy','rb') as f:
    rank = np.load(f,allow_pickle=True)

for r in range(len(rank)):
    print("Rank",r+1,"Num:",rank[r].num,"Mu:",rank[r].rate.mu)
print("Total:",len(rank))