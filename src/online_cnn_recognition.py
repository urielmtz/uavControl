# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 14:48:22 2018

@author: Uriel Martinez-Hernandez

Description: CNN classifier for active visual object exploration and recognition
"""

import roslib
import Image
from PIL import Image                                                            
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float32
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

model = keras.models.load_model('/home/adrian/Documents/camera_data/models/model_segmented_24022018_0001.h5')

graph = tf.get_default_graph()

namesList = ['box', 'circle', 'cup', 'pen','rectangle']  #[object 0, object 1, object 2, object 3, object 4]
namesListPlot = ('box', 'circle', 'cup', 'pen','rectangle')  #[object 0, object 1, object 2, object 3, object 4]
y_pos = np.arange(len(namesListPlot))

x_test = np.zeros((1, imHeight, imWidth, 1))
test_img = np.zeros((1,71,128,1))

y_test = np.zeros((len(x_test),));
y_test = keras.utils.to_categorical(y_test, len(namesList));

plt.ion()

pub = rospy.Publisher('cnn_probability', Float32, queue_size=1)

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
        pil_image = cv2.resize(pil_image, (imWidth, imHeight))
        x_test[0,:,:,0] = np.array(pil_image, 'f')
      
        test_img[0] = x_test[0]
      
        output = model.predict(test_img, batch_size=1)
        print(output)
        recognisedObject = np.argmax(output[0,:], axis=1)

		if( recognisedObject == 0 ):
            print("Object: box");
        elif( recognisedObject == 1 ):
            print("Object: circle");        
        elif( recognisedObject == 2 ):
            print("Object: cup");        
        elif( recognisedObject == 3 ):
            print("Object: pen");        
        else:
            print("Object: rectangle");


        probabilitiesPlot = output[0,:];
        maxProbabilityValue = np.max(output);

        pub.publish(maxProbabilityValue);
        

        plt.clf()
        plt.bar(y_pos, probabilitiesPlot, align='center', alpha=0.5)
        plt.xticks(y_pos, namesListPlot)
        plt.ylabel('probability')
        plt.title('Active visual object recognition')
        plt.pause(0.05);

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