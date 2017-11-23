import cv2
import numpy as np
from networktables import NetworkTable
class Vision:
    def __init__(self,calibration=False):
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
            * centers_x : A list of the x values of all centers, empty until the find_center() function is called.
            * centers_y : A list of the y values of all centers, empty until the find_center() function is called.
            * center : The average point of all centers of all contours.
        """
        self.kernel = np.array([
            [1, 1, 1],
            [1, 1, 1],
            [1, 1, 1]
        ])
        self.frame = np.load('map_vision.npy')
        self.frame2=self.frame.copy()
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.set_range()
        self.mask = cv2.inRange(self.hsv, self.lower_range, self.upper_range)
        _, contours, _ = cv2.findContours(self.mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        self.contours=list(contours)
        self.hulls = []
        self.centers_x = []
        self.centers_y = []
        self.center = (0, 0)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.calibration=calibration
        if not self.calibration:
            file=open('function.val','r')
            funnction=file.read()
            self.get_distance=lambda x:eval(funnction)#if we are not in calibration mode take the function from the file
            file.close()
        else:
            self.input=0#initiating the input variable
            self.dist_cal=[]#creates an empty list of
            self.area_cal=[]#both distance and area
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
        self.set_item("Find center", self.find_center_b)

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
        #file = open("Ace.acpf", 'r')
        #exec(file.read())
        #file.close()
        self.lower_range=(0,255,0)
        self.upper_range=(0,255,0)
    def draw_contours(self):
        # Draws contours on the frame, if asked so on SmartDashboard
        if len(self.contours) > 0 and self.get_item("Draw contours", self.draw_contours_b):
            for x in range(0, len(self.contours)):
                cv2.drawContours(self.frame2, self.contours[x], -1, (255, 0, 0), 3)
                # Draws hulls on the frame, if asked so on SmartDashboard
                #if len(self.hulls) > 0 and self.get_item("Draw hulls", self.draw_hulls_b):
                    #defects = cv2.convexityDefects(self.contours[x], self.hulls[x])
                    #for i in range(defects.shape[0]):
                    #    s, e, f, d = defects[i, 0]
                    #    start = tuple(self.contours[x][s][0])
                    #    end = tuple(self.contours[x][e][0])
                    #    far = tuple(self.contours[x][f][0])
                    #    cv2.line(self.frame2, start, end, [0, 255, 0], 2)
                    #    cv2.circle(self.frame2, far, 5, [0, 0, 255], -1)

    def dirode(self):
        # Dialates and erodes the mask to reduce static and make the image clearer
        cv2.dilate(self.mask, self.kernel, iterations = self.get_item("DiRode iterations", self.dirode_iterations_i))
        cv2.erode(self.mask, self.kernel, iterations = self.get_item("DiRode iterations", self.dirode_iterations_i))

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

    def find_center(self):
        # Finds the average of all centers of all contours
        self.centers_x.clear()
        self.centers_y.clear()
        if self.get_item("Find center", self.find_center_b) and len(self.contours) > 0:
            for c in self.contours:
                (x, y), _ = cv2.minEnclosingCircle(c)
                self.centers_x.append(x)
                self.centers_y.append(y)
            self.center = (int((sum(self.centers_x) / len(self.centers_x))), (int(sum(self.centers_y) / len(self.centers_y))))
            cv2.putText(self.frame2, "o {}".format(self.center), self.center, self.font, 0.5, 255)

    def get_contours(self):
        # Executes a command line from SmartDashboard
        command = self.get_item("Command", self.command_s)
        functions = command.split(";")
        if len(functions) > 0:
            for fun in functions:
                if not fun == "":
                    exec("self."+fun)

    def get_angle(self):
        self.angle = self.center[0]*45 / 320 -45
        cv2.putText(self.frame2, "Angle: {}".format(self.angle), (5, 15), self.font, 0.5, 255)

    def numbers_input(self,key):
        """
        uses the numpad as a way of writing distance
        :param key: gets keyboard key from the process
        :return:
        """
        if key in range(48,58):#48-58 are the possible numbers
            self.input *= 10
            self.input += key-48
        if key is 13: #13 is Enter
            self.dist_cal.append(self.input)
            self.total_area=0
            for c in self.contours:
                self.total_area+=cv2.contourArea(c)
            self.area_cal.append(self.total_area)
            self.input=0
        if key is 8: #8 is backspace
            if self.input is 0:
                self.dist_cal.pop(len(self.dist_cal)-1)#if the number is 0 remove the last dist inserted
                self.area_cal.pop(len(self.area_cal)-1)#if the number is 0 remove the last area inserted
            self.input/=10
            self.input=int(self.input)

    def create_poly(self,deg):
        """
        this function creates a function of degree deg
        :param deg: the degree of the funtion
        :return: writes to a file the wanted function
        """
        polyfit=np.polyfit(self.area_cal,self.dist_cal,deg=deg)
        polyfit=list(polyfit)
        string='0'
        for i in polyfit:
            string += '+'+str(i)+'*x**'+str((deg-polyfit.index(i)))
        file=open('function.val','w')
        file.write(string)
        file.close()
key=-1
vision = Vision()
while True:
    # Repeatedly reads the next frame, turns it into an HSV mask, and finds contours
    if key == 2490368:
        vision.frame=cv2.erode(vision.frame,vision.kernel)
    elif key == 2621440:
        vision.frame=cv2.dilate(vision.frame,vision.kernel)
    vision.frame2=vision.frame.copy()
    vision.hsv = cv2.cvtColor(vision.frame, cv2.COLOR_BGR2HSV)
    vision.mask = cv2.inRange(vision.frame, vision.lower_range, vision.upper_range)
    _, contours, _ = cv2.findContours(vision.mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    vision.contours=list(contours)
    vision.get_contours()
    vision.draw_contours()
    vision.find_center()
    vision.get_angle()
    key = cv2.waitKeyEx(1)
    if vision.calibration is True:
        vision.numbers_input(key)
        cv2.putText(vision.frame2,"input: "+str(vision.input),(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(vision.frame2,"area: "+str(vision.area_cal),(50,100), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(vision.frame2,"distance: "+str(vision.dist_cal),(50,150), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
        if key is ord('p') and vision.calibration:
            vision.create_poly(5)#5 is the function's deg

    else:
        vision.total_area=0
        for c in vision.contours:
            vision.total_area+=cv2.contourArea(c)
        vision.distance=vision.get_distance(vision.total_area)
        cv2.putText(vision.frame2,"distance: "+str(vision.distance),(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)

    cv2.imshow("Frame", vision.frame2)
    cv2.imshow("Mask", vision.mask)

    if key == ord("q"):
        break