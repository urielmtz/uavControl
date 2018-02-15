# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 14:48:22 2018

@author: adrian
"""

import roslib
#roslib.load_manifest('/home/adrian/catkin_ws/src/uavControl/package')
import Image
from PIL import Image                                                            
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
import scipy.io as sio
import numpy as np
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras.optimizers import SGD
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
import matplotlib.cm as cm
import numpy.ma as ma
import glob
#import Image


np.random.seed(7)

imHeight = 71
imWidth = 128

model = keras.models.load_model('/home/adrian/Documents/camera_data/models/model_segmented_12022018_0001.h5')

graph = tf.get_default_graph()

namesList = ['cup', 'circle', 'box']

x_test = np.zeros((1, imHeight, imWidth, 1))
test_img = np.zeros((1,71,128,1))

y_test = np.zeros((len(x_test),));
y_test = keras.utils.to_categorical(y_test, len(namesList));


class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_processed",Image,self.callback)



  def callback(self,data):
    try:
      global graph
      with graph.as_default():
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        pil_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)      
        #print(pil_image.shape)
        pil_image = cv2.resize(pil_image, (imWidth, imHeight))
        #print(pil_image.shape)
        x_test[0,:,:,0] = np.array(pil_image, 'f')
        #print(x_test.shape)
      
        test_img[0] = x_test[0]
        #print(test_img.shape)
      
        output = model.predict(test_img, batch_size=1)
        print(output)
        print(np.argmax(output, axis=1))

    except CvBridgeError as e:
      print(e)

    cv2.imshow("Python image", cv_image)
    cv2.waitKey(3)


def main(args):
  ic = image_converter()
  rospy.init_node('image_classifier', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)