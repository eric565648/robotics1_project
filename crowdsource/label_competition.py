# This script is for the tournement

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

SETSIZE=1000
COMPNUM_1=5
COMPNUM_2=5
COMPNUM_3=10
COMPNUM=[5,5,10]
TOTAL=5000

dataset = []
compset = []
rank = []

# generate dataset
for i in range(SETSIZE):
    this_data = MData(i+1)
    dataset.append(this_data)
compset = dp(dataset)

# set up pygame gui
v = cv2.VideoCapture("/media/eric/Transcend/motion_lib/motion_data/1.avi")
suc1, video_image = v.read()
FPS = v.get(cv2.CAP_PROP_FPS)
WIDTH = video_image.shape[1]
HEIGHT = video_image.shape[0]
MARGIN=10
LOMARGIN=30
window = pg.display.set_mode((WIDTH+MARGIN,HEIGHT/2+LOMARGIN))
clock = pg.time.Clock()

# Counter
counter = 0

for rt in [0,1,2]:
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

    # for cp in cporder:
    #     print("p1:",cp['p1'],",p2:",cp['p2'])
    shuford=np.arange(len(cporder))
    np.random.shuffle(shuford)
    cporder_shuffle=dp(cporder)
    for shufi in range(len(shuford)):
        cporder_shuffle[shufi] = cporder[shuford[shufi]]
    # for cp in cporder_shuffle:
    #     print("p1:",cp['p1'],",p2:",cp['p2'])

    # compare and apply TrueSkill
    cporder_count=0
    for cp in cporder_shuffle:
        counter += 1
        cporder_count += 1
        p1=cp['p1']
        p2=cp['p2']

        print("================")
        print("Step: ",counter,",",TOTAL-counter,"to go. This count:",cporder_count)
        print(compset[p1].num,"v.s.",compset[p2].num)

        v1 = cv2.VideoCapture("/media/eric/Transcend/motion_lib/motion_data/"+str(compset[p1].num)+".avi")
        v2 = cv2.VideoCapture("/media/eric/Transcend/motion_lib/motion_data/"+str(compset[p2].num)+".avi")
        LEFT = False
        RIGHT = False
        run=True
        while run:
            clock.tick(FPS)
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    run = False
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_LEFT:
                        LEFT=True
                    if event.key == pg.K_RIGHT:
                        RIGHT=True
                else:
                    if LEFT or RIGHT:
                        run = False
            
            suc1, vimg1 = v1.read()
            suc2, vimg2 = v2.read()
            if suc1:
                video_surf1 = pg.image.frombuffer(
                    vimg1.tobytes(), vimg1.shape[1::-1], "BGR")
                video_surf1 = pg.transform.scale(video_surf1,(WIDTH/2,HEIGHT/2))
            else:
                v1.set(cv2.CAP_PROP_POS_FRAMES, 0)
            if suc2:
                video_surf2 = pg.image.frombuffer(
                    vimg2.tobytes(), vimg2.shape[1::-1], "BGR")
                video_surf2 = pg.transform.scale(video_surf2,(WIDTH/2,HEIGHT/2))
            else:
                v2.set(cv2.CAP_PROP_POS_FRAMES, 0)

            if LEFT:
                pg.draw.rect(video_surf1, (255,0,0), pg.Rect(0, 0, WIDTH/2, HEIGHT/2),  10)

            if RIGHT:
                pg.draw.rect(video_surf2, (255,0,0), pg.Rect(0, 0, WIDTH/2, HEIGHT/2),  10)

            window.blit(video_surf1, (0, LOMARGIN/2))
            window.blit(video_surf2, (WIDTH/2+MARGIN, LOMARGIN/2))
            pg.display.flip()

        if LEFT and RIGHT:
            nr1,nr2=tk.rate_1vs1(compset[p1].rate,compset[p2].rate, drawn=True)
        elif LEFT:
            nr1,nr2=tk.rate_1vs1(compset[p1].rate,compset[p2].rate)
        elif RIGHT:
            nr2,nr1=tk.rate_1vs1(compset[p2].rate,compset[p1].rate)
        # assign new rating
        compset[p1].rate=nr1
        compset[p2].rate=nr2

        with open('/media/eric/Transcend/motion_lib/motion_data/compset.npy','wb') as f:
            np.save(f,compset)
        with open('/media/eric/Transcend/motion_lib/motion_data/cporder.npy','wb') as f:
            np.save(f,cporder)
        with open('/media/eric/Transcend/motion_lib/motion_data/order_count.npy','wb') as f:
            np.save(f,np.array([cporder_count]))
        with open('/media/eric/Transcend/motion_lib/motion_data/rank.npy','wb') as f:
            np.save(f,rank)
    
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
with open('/media/eric/Transcend/motion_lib/motion_data/rank.npy','wb') as f:
    np.save(f,rank)
for r in range(len(rank)):
    print("Rank",r+1,"Num:",rank[r].num,"Mu:",rank[r].rate.mu)
print("Total:",len(rank))