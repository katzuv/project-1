"""
All functions below filter contours based on a trait and a range set in SmartDashboard
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
"""