#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String
# from PIL import Image
# import pyscreenshot as ImageGrab
from fastgrab import screenshot
import cv2

class Recorder(object):
    def __init__(self) -> None:
        super().__init__()

        self.width = 1350
        self.height = 850

        self.rec_dur = 20
        self.rec_srv = rospy.Subscriber("screen_record",String,self.rec_srv_cb,queue_size=1)

        self.cap_r = 30
        self.cap_rate = rospy.Rate(self.cap_r)

        rospy.set_param('record_flag', True)

    def rec_srv_cb(self, msg):

        rospy.set_param('record_flag', False)

        # display screen resolution, get it from your OS settings
        SCREEN_SIZE = (self.width, self.height)
        # define the codec
        fourcc = cv2.VideoWriter_fourcc(*"X264")
        # create the video write object
        path = msg.data
        out = cv2.VideoWriter(path+".avi", fourcc, self.cap_r, (SCREEN_SIZE))

        rospy.set_param('record_switch', True)
        record_switch = True
        st = rospy.Time.now()
        img_list = []
        while (rospy.Time.now()-st).to_sec()<self.rec_dur:
            # make a screenshot
            img = screenshot.Screenshot().capture(bbox=(90, 130, self.width, self.height))
            img_list.append(img)

            record_switch = rospy.get_param("record_switch")
            if record_switch == False:
                break

            self.cap_rate.sleep()
        
        discard = 10
        i = 0
        for img in img_list:
            i+=1
            # discard first few frame
            if i<=discard:
                continue

            # convert these pixels to a proper numpy array to work with OpenCV
            # frame = np.array(img)
            # convert colors from BGR to RGB
            frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # write the frame
            out.write(frame)
        
        out.release()

        rospy.set_param('record_flag', True)

def main():

    rospy.init_node('screen_record')
    re = Recorder()

    rospy.spin()

if __name__ == '__main__':
    main()