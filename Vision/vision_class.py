import cv2
import numpy as np
from networktables import NetworkTable
class Vision:
    def __init__(self):
        # Initialize camera and first frame reading
        self.cam = cv2.VideoCapture(0)
        _, self.frame = self.cam.read()
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.set_range()
        self.mask = cv2.inRange(self.hsv, self.lower_range, self.upper_range)
        _, self.contours, _ = cv2.findContours(self.mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Initialize SmartDashboard
        NetworkTable.initialize(server="roborio-{}-frc.local".format(5987))
        self.table = NetworkTable.getTable('SmartDashboard')
        self.set_item("Command", "")
        self.set_item("Draw contours", False)
        self.set_item("Draw hulls", False)
        self.set_item("DiRode iterations", 3)


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
        # Retrieves the range written in "Ace" which was written there by Range Finder
        file = open("Ace.acpf", 'r')
        exec(file.read())
        file.close()

    def draw_contours(self):
        # Draws contours on the frame, if asked so on SmartDashboard
        if len(self.contours) > 0 and self.get_item("Draw contours", False):
            for c in self.contours:
                cv2.drawContours(self.frame, c, -1, (255, 255, 255), 4)
        # Draws hulls on the frame, if asked so on SmartDashboard
        if len(self.hulls) > 0 and self.get_item("Draw hulls", False):
            for h in self.hulls:
                cv2.drawContours(self.frame, h, -1, (255, 255, 255), 4)

    def dirode(self):
        # Dialates and erodes the mask to reduce static and make the image clearer
        kernel = np.array([
            [0, 1, 0],
            [1, 1, 1],
            [0, 1, 0]
        ])
        cv2.dilate(self.mask, kernel, iterations = self.get_item("DiRode iterations", 3))
        cv2.erode(self.mask, kernel, iterations = self.get_item("DiRode iterations", 3))

    # All functions below filter contours based on a trait and a range set in SmartDashboard
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
        # Adds a list of hulls, which can be drawn like contours
        self.hulls = []
        for c in self.contours:
            if not (u > float(cv2.contourArea(c)) / cv2.contourArea(cv2.convexHull(c)) > l):
                self.contours.remove(c)
                # Adds a hull to the list only if it fits our parameters
                self.hulls.append(cv2.convexHull(c))

    def get_contours(self):
        # Executes a command line from SmartDashboard
        command = self.get_item("Command", "")
        functions = command.split(";")
        try:
            for fun in functions:
                exec(fun)
        except:
            self.set_item("Command", "Not valid")

    def __main__(self):
        # Repeatedly reads the next frame, turns it into an HSV mask, and finds contours
        _, self.frame = self.cam.read()
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(self.hsv, self.lower_range, self.upper_range)
        _, self.contours, _ = cv2.findContours(self.mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.get_contours()
        self.draw_contours()

        cv2.imshow("Frame", self.frame)
        cv2.imshow("Mask", self.mask)
vision = Vision()
vision.__init__()
while True:
    vision.__main__()
    key = cv2.waitKey(1)
    if key == ord("q"):
        break