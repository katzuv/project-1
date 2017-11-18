import cv2
import numpy as np
from networktables import NetworkTable

class cam:
    def __init__(self):
        # Initialize camera and first frame reading
        self.cam = cv2.VideoCapture(0)
        _, self.frame = cam.read()
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
        des_area = self.get_item("Desired Area")
        self.upper_range = (255, 255, 255)
        self.lower_range = (0, 0, 0)
        # Needs the find contour functions

    def get_contour(self):
        # Master function to getting all the details about the biggest (main) contour
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_range, self.upper_range)
        _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            the_one_and_only = max(contours, key=cv2.contourArea)

        def area():
            return cv2.contourArea(the_one_and_only)
        def bounding_rect():
            return cv2.boundingRect(the_one_and_only)
        def bounding_circ():
            return cv2.minEnclosingCircle(the_one_and_only)
        def extent():
            return float(area()) / bounding_rect()
        def hull():
            return cv2.convexHull(the_one_and_only)
        def hull_area():
            return cv2.contourArea(hull())
        def solidity():
            return float(area()) / hull_area()
