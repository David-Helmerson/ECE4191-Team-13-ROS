import numpy as np
import imuils
from imuils import paths
import cv2


class distance(image):

    def _init_(self,image,init_image,marble_radius,init_distance):
        self.image = image
        self.init_image = init_image
        self.marble_radius = marble_radius #to be set
        self.init_distance = init_distance #to be set
        self.distance = None
        self.focul_length = None
        self.init_circle = None
        self.circle = None
        self.x = None
        self.y = None
        self.r = None

    def initialization(self):
        self.init_image = cv2.cvtColor(self.init_image,cv2.COLOR_BGR2GRAY)
        self.init_image = cv2.GaussianBlur(self.init_image, (5, 5), 0)
        self.init_image = cv2.Canny(self.init_image, 35, 125)
        self.init_circle = cv.HoughCircles(self.init_image, cv.HOUGH_GRADIENT, 1, 30, param1=50, param2=30, minRadius=0, maxRadius=50)
        self.focul_length = self.init_circle[0,:][2] * self.init_distance / self.marble_radius


    def imageprocessing(self):
        #convert the input image to gray and edged image
        self.image = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        self.image = cv2.GaussianBlur(self.image, (5, 5), 0)
        self.image = cv2.Canny(self.image, 35, 125)
        

        #find the contour of the image to identify the objects
        #param1 param2

        self.circle = cv.HoughCircles(self.image, cv.HOUGH_GRADIENT, 1, 30, param1=50, param2=30, minRadius=0, maxRadius=50)
        self.x = self.circle[0,:][0]
        self.y = self.circle[0,:][1]
        self.r = self.circle[0,:][2]

    def distance_cal(self,self.r):
        self.distance = self.marble_radius *  self.focul_length / self.r
        return self.distance









    

