import cv2
import numpy as np
from networktables import NetworkTable

class Vision:
    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        _, self.frame = self.cam.read()
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(self.hsv, self.lower_range, self.upper_range)
        _, self.contours, _ = cv2.findContours(self.mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Initialize SmartDashboard
        NetworkTable.initialize(server="roborio-{}-frc.local".format(5987))
        self.table = NetworkTable.getTable('SmartDashboard')

    def set_item(self, key, value):
        # Add a value to SmartDashboard
        value_type = type(value)
        if value_type is str:
            self.table.putString(key, value)
        elif value_type is int or value_type is float:
            self.table.putNumber(key, value)
        elif value_type is bool:
            self.table.putBoolean(key, value)

    def get_item(self, key, default_value):
        # Get a value from SmartDashboard
        try:
            res = self.table.getString(key, default_value)
        except:
            try:
                res = self.table.getNumber(key, default_value)
            except:
                res = self.table.getBoolean(key, default_value)
        return res

    def set_range(self):
        # Find the HSV range of an object in the center
        self.upper_range = (255, 255, 255)
        self.lower_range = (0, 0, 0)
        # Needs the find contour functions

    def draw_contours(self):
        if len(self.contours) > 0 and self.get_item("Draw contours", False):
            for c in self.contours:
                cv2.drawContours(self.frame, c, -1, (255, 255, 255), 4)
        if len(self.hulls) > 0 and self.get_item("Draw hulls", False):
            for h in self.hulls:
                cv2.drawContours(self.frame, h, -1, (255, 255, 255), 4)

    def dirode(self):
        kernel = np.zeros((5, 5), dtype = np.uint8)
        cv2.dilate(self.mask, kernel, iterations = self.get_item("DiRode iterations", 3))
        cv2.erode(self.mask, kernel, iterations = self.get_item("DiRode iterations", 3))

    def area(self, l, u):
        for c in self.contours:
            if not (u > cv2.contourArea(c) > l):
                self.contours.remove(c)
    def bounding_rect(self, l, u):
        for c in self.contours:
            if not (u > cv2.boundingRect(c) > l):
                self.contours.remove(c)
    def bounding_circ(self, l, u):
        for c in self.contours:
            if not (u > cv2.minEnclosingCircle(c) > l):
                self.contours.remove(c)
    def extent(self, l, u):
        for c in self.contours:
            if not (u > float(cv2.contourArea(c)) / cv2.boundingRect(c) > l):
                self.contours.remove(c)
    def hull(self, l, u):
        self.hulls = []
        for c in self.contours:
            if not (u > float(cv2.contourArea(c)) / cv2.contourArea(cv2.convexHull(c)) > l):
                self.contours.remove(c)
                self.hulls.append(cv2.convexHull(c))

    def get_contours(self):
        try:
            command = self.get_item("command")
        except:
            command = ''
            self.set_item("command", "not valid")
        functions = command.split(";")
        for fun in functions:
            exec(fun)

    def __main__(self):
        _, self.frame = self.cam.read()
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(self.hsv, self.lower_range, self.upper_range)
        _, self.contours, _ = cv2.findContours(self.mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.get_contours()
        self.draw_contours()