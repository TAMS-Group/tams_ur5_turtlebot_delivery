from __future__ import absolute_import
import cv2
import numpy as np

image = np.zeros((100,80,3), np.uint8)
gray_img=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
