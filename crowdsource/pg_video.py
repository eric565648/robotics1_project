import pygame
import cv2
import time

video1 = cv2.VideoCapture("/media/eric/Transcend/motion_lib/motion_data/1.avi")
video2 = cv2.VideoCapture("/media/eric/Transcend/motion_lib/motion_data/2.avi")
suc1, video_image = video1.read()
fps = video1.get(cv2.CAP_PROP_FPS)

print(video_image.shape)
print(video_image.shape[1::-1])

MARGIN=10
LOMARGIN=30

window = pygame.display.set_mode((video_image.shape[1]+MARGIN,video_image.shape[0]/2+LOMARGIN))
clock = pygame.time.Clock()

for i in [0,1,2,3]:
    video1 = cv2.VideoCapture("/media/eric/Transcend/motion_lib/motion_data/"+str(2*i+1)+".avi")
    video2 = cv2.VideoCapture("/media/eric/Transcend/motion_lib/motion_data/"+str(2*i+2)+".avi")
    LEFT = False
    RIGHT = False
    run=True
    while run:
        clock.tick(fps)
        st = time.perf_counter_ns()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    LEFT=True
                if event.key == pygame.K_RIGHT:
                    RIGHT=True
            else:
                if LEFT or RIGHT:
                    run = False
        
        suc1, vimg1 = video1.read()
        suc2, vimg2 = video2.read()
        if suc1:
            video_surf1 = pygame.image.frombuffer(
                vimg1.tobytes(), vimg1.shape[1::-1], "BGR")
            video_surf1 = pygame.transform.scale(video_surf1,(vimg1.shape[1]/2,vimg1.shape[0]/2))
        else:
            video1.set(cv2.CAP_PROP_POS_FRAMES, 0)
        if suc2:
            video_surf2 = pygame.image.frombuffer(
                vimg2.tobytes(), vimg2.shape[1::-1], "BGR")
            video_surf2 = pygame.transform.scale(video_surf2,(vimg2.shape[1]/2,vimg2.shape[0]/2))
        else:
            video2.set(cv2.CAP_PROP_POS_FRAMES, 0)

        if LEFT:
            pygame.draw.rect(video_surf1, (255,0,0), pygame.Rect(0, 0, video_image.shape[1]/2, video_image.shape[0]/2),  2)

        if RIGHT:
            pygame.draw.rect(video_surf2, (255,0,0), pygame.Rect(0, 0, video_image.shape[1]/2, video_image.shape[0]/2),  2)

        window.blit(video_surf1, (0, LOMARGIN/2))
        window.blit(video_surf2, (video_image.shape[1]/2+MARGIN, LOMARGIN/2))
        pygame.display.flip()
        et = time.perf_counter_ns()
        if 1e+9/(et-st)<30:
            print(1e+9/(et-st))

pygame.quit()
exit()