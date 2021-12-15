# This script is a testing for the tournement

import numpy as np
import math
from copy import deepcopy as dp
import trueskill as tk

# toy_set = np.arange(1000)

class MData:
    def __init__(self,num,score) -> None:
        self.num = num
        self.score = score
        self.comp = np.array([])
        self.rate=tk.Rating()
    def set_comp(self,comp):
        self.comp = np.append(self.comp,comp)
        return True
    def set_rate(self,rate):
        self.rate=rate

SETSIZE=5
COMPNUM_1=4
COMPNUM_2=5
COMPNUM_3=10
COMPNUM=[4,5,10]

toyset = []
compset = []
rank = []

# generate toy dataset
for i in range(SETSIZE):
    this_data = MData(i,i)
    toyset.append(this_data)
compset = dp(toyset)


for rt in [0]:
    print('Round',rt+1)

    # Round Start
    this_rank = []

    # Set up the matching order
    while True:
        cporder = []
        # restart flag
        restart = False
        # match competitor
        remain_comp=np.arange(len(compset))
        for i in range(len(compset)):
            while True:
                if len(compset[i].comp)>=COMPNUM[rt]:
                    break

                # trigger restart if only myself as competitor
                if len(remain_comp)==1:
                    if remain_comp[0]==i:
                        restart=True
                        break
                # choose competitor
                while True:
                    cpnum=np.random.choice(remain_comp)
                    # print("cpnum",cpnum)
                    # print("i",i)
                    if cpnum != i:
                        if not cpnum in compset[i].comp:
                            break
                if len(compset[cpnum].comp)<COMPNUM[rt]:
                    # set competitor
                    compset[i].set_comp(cpnum)
                    compset[cpnum].set_comp(i)
                    cp={'p1':i,'p2':cpnum}
                    cporder.append(cp)
                    if len(compset[i].comp)>=COMPNUM[rt]:
                        remain_comp=np.delete(remain_comp,np.argwhere(remain_comp==i)[0,0])
                    if len(compset[cpnum].comp)>=COMPNUM[rt]:
                        remain_comp=np.delete(remain_comp,np.argwhere(remain_comp==cpnum)[0,0])
                else:
                    remain_comp=np.delete(remain_comp,np.argwhere(remain_comp==cpnum)[0,0])
                    # print(len(remain_comp))
        if not restart:
            # for cp in cporder:
            #     print("p1:",cp['p1'],",p2:",cp['p2'])
            print("Comp Times:",len(cporder))
            break

        # Someone has no match!!
        # restart distributing competitor
        print('restart')
        for cp in compset:
            cp.comp=np.array([])

    # check if all data compare exact times
    # comptime=np.zeros(len(compset))
    # for cp in cporder:
    #     comptime[cp['p1']] += 1
    #     comptime[cp['p2']] += 1
    # print(comptime)

    shuford=np.arange(len(cporder))
    np.random.shuffle(shuford)
    cporder_shuffle=dp(cporder)
    for shufi in range(len(shuford)):
        cporder_shuffle[shufi] = cporder[shuford[shufi]]

    # compare using TrueSkill
    for cp in cporder_shuffle:
        p1=cp['p1']
        p2=cp['p2']
        print("p1,p2:",p1,p2)
        if compset[p1].score>compset[p2].score:
            nr1,nr2=tk.rate_1vs1(compset[p1].rate,compset[p2].rate)
        else:
            nr2,nr1=tk.rate_1vs1(compset[p2].rate,compset[p1].rate)
        # assign new rating
        compset[p1].rate = nr1
        compset[p2].rate=nr2
    
    # Rank all competitor in this round
    this_rank.append(compset[0])
    for i in range(1,len(compset)):
        ins_id=len(this_rank)
        for r in range(len(this_rank)):
            if compset[i].rate.mu > this_rank[r].rate.mu:
                ins_id=r
                break
            elif compset[i].rate.mu == this_rank[r].rate.mu:
                if compset[i].rate.sigma < this_rank[r].rate.sigma:
                    ins_id=r
                    break
        this_rank.insert(ins_id,compset[i])
    
    # put all competitor back to the total rank
    if rt==0:
        rank=dp(this_rank)
        compset = dp(rank[0:int(SETSIZE/2)])
        for cp in compset:
            cp.comp=np.array([])
    elif rt==1:
        rank[0:int(SETSIZE/2)]=dp(this_rank)
        compset = dp(rank[0:int(SETSIZE/4)])
        for cp in compset:
            cp.comp=np.array([])
    elif rt==2:
        rank[0:int(SETSIZE/4)]=dp(this_rank)
    
# show rank
for r in range(len(rank)):
    print("Rank",r+1,"Num:",rank[r].num,"Mu:",rank[r].rate.mu,"GT:",rank[r].score)
print("Total:",len(rank))