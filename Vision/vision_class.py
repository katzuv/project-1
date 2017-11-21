import cv2
import numpy as np
from networktables import NetworkTable
class Vision:
    def __init__(self):
        """
        Summary: Start camera, read and analyze first frame.
        Parameters:
            * cam : The camera the code will use. Set to 0 if there's only one connected, check ports if more than one.
            Usually, a USB attached camera will have a bigger port than native ones.
            * frame : A numpy array. The frame the camera captured. All contours and hulls will be drawn on it.
            * hsv : The conversion of the values in frame from BGR to HSV, since our code operates on it.
            * mask : The pixels found within a preset range.
            * contours : A numpy array, converted to a list for easy of use. Stores the x and y values of all the contours.
            * hulls : A list of hulls, empty until the hull() function is called.
        """
        self.cam = cv2.VideoCapture(1)
        _, self.frame = self.cam.read()
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.set_range()
        self.mask = cv2.inRange(self.hsv, self.lower_range, self.upper_range)
        _, contours, _ = cv2.findContours(self.mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        self.contours=list(contours)
        self.hulls = []
        """
        Summary: Get SmartDashboard. 
        # Currently unavailable. Instead, create and read a file where all values are stored.
        # BTW, why is this one a different color?
        """
        NetworkTable.initialize(server="roborio-{}-frc.local".format(5987))
        self.table = NetworkTable.getTable('SmartDashboard')
        # NetworkTable.initialize(server='192.168.13.75')
        # self.table = NetworkTable.getTable("ACoolTable")
        file = open('Values.val','r')
        exec(file.read())
        file.close()
        self.set_item("Command", self.command_s)
        self.set_item("Draw contours", self.draw_contours_b)
        self.set_item("Draw hulls", self.draw_hulls_b)
        self.set_item("DiRode iterations", self.dirode_iterations_i)

    def set_item(self, key, value):
        """
        Summary: Add a value to SmartDashboard.

        Parameters:
            * key : The name the value will be stored under and displayed.
            * value : The information the key will hold.
            * value_type : The type of the value it recieved (string, integer, boolean, etc.).
        """
        value_type = type(value)
        if value_type is str:
            self.table.putString(key, value)
        elif value_type is int or value_type is float:
            self.table.putNumber(key, value)
        elif value_type is bool:
            self.table.putBoolean(key, value)

    def get_item(self, key, default_value):
        """
        Summary: Get a value from SmartDashboard.

        Parameters:
            * key : The name the value is stored under.
            * default_value : The value returned if key holds none.
        """
        try:
            res = self.table.getString(key, default_value)
        except:
            try:
                res = self.table.getNumber(key, default_value)
            except:
                res = self.table.getBoolean(key, default_value)
        return res

    def set_range(self):
        # Retrieves the range written in "Ace" which was written there by Range Finder 3.0
        file = open("Ace.acpf", 'r')
        exec(file.read())
        file.close()

    def draw_contours(self):
        # Draws contours on the frame, if asked so on SmartDashboard
        if len(self.contours) > 0 and self.get_item("Draw contours", self.draw_contours_b):
            for x in range(0, len(self.contours)):
                cv2.drawContours(self.frame, self.contours[x], -1, (255, 0, 0), 3)
                # Draws hulls on the frame, if asked so on SmartDashboard
                if len(self.hulls) > 0 and self.get_item("Draw hulls", self.draw_hulls_b):
                    defects = cv2.convexityDefects(self.contours[x], self.hulls[x])
                    for i in range(defects.shape[0]):
                        s, e, f, d = defects[i, 0]
                        start = tuple(self.contours[x][s][0])
                        end = tuple(self.contours[x][e][0])
                        far = tuple(self.contours[x][f][0])
                        cv2.line(self.frame, start, end, [0, 255, 0], 2)
                        cv2.circle(self.frame, far, 5, [0, 0, 255], -1)

    def dirode(self):
        # Dialates and erodes the mask to reduce static and make the image clearer
        kernel = np.array([
            [0, 1, 0],
            [1, 1, 1],
            [0, 1, 0]
        ])
        cv2.dilate(self.mask, kernel, iterations = self.get_item("DiRode iterations", self.dirode_iterations_i))
        cv2.erode(self.mask, kernel, iterations = self.get_item("DiRode iterations", self.dirode_iterations_i))

    # All functions below filter contours based on a trait and a range set in SmartDashboard
    def area(self, l, u):
        if len(self.contours) > 0:
            possible_fit = []
            for c in self.contours:
                if u > cv2.contourArea(c) > l:
                    possible_fit.append(c)
            self.contours=possible_fit
    def bounding_rect(self, l, u):
        possible_fit = []
        if len(self.contours) > 0:
            for c in self.contours:
                if u > cv2.boundingRect(c) > l:
                    possible_fit.append(c)
            self.contours=possible_fit
    def bounding_circ(self, l, u):
        possible_fit = []
        if len(self.contours) > 0:
            for c in self.contours:
                if u > cv2.minEnclosingCircle(c) > l:
                    possible_fit.append(c)
            self.contours=possible_fit
    def extent(self, l, u):
        possible_fit = []
        for c in self.contours:
            _, _, w, h = cv2.boundingRect(c)
            rect_area = w*h
            if u > cv2.contourArea(c)/rect_area > l:
                possible_fit.append(c)
        self.contours = possible_fit
    def hull(self, l, u):
        # Adds a list of hulls, which can be drawn like contours
        self.hulls.clear()
        possible_fit = []
        for c in self.contours:
            if (u > cv2.contourArea(c) / cv2.contourArea(cv2.convexHull(c)) > l):
                possible_fit.append(c)
                # Adds a hull to the list only if it fits our parameters
                self.hulls.append(cv2.convexHull(c, returnPoints=False))
        self.contour = possible_fit

    def get_contours(self):
        # Executes a command line from SmartDashboard
        command = self.get_item("Command", self.command_s)
        functions = command.split(";")
        if len(functions) > 0:
            for fun in functions:
                exec("self."+fun)

vision = Vision()
while True:
    # Repeatedly reads the next frame, turns it into an HSV mask, and finds contours
    _, vision.frame = vision.cam.read()
    vision.hsv = cv2.cvtColor(vision.frame, cv2.COLOR_BGR2HSV)
    vision.mask = cv2.inRange(vision.hsv, vision.lower_range, vision.upper_range)
    _, contours, _ = cv2.findContours(vision.mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    vision.contours=list(contours)
    vision.get_contours()
    vision.draw_contours()
    cv2.imshow("Frame", vision.frame)
    cv2.imshow("Mask", vision.mask)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break