#! /usr/bin/env python
import time
import rospy
import math
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from vitarana_drone.msg import *
from std_msgs.msg import String
from std_msgs.msg import Float32
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Edrone():
    """ Edrone Class """

    def __init__(self,start = None,end = None,boundary_points = None):

        rospy.init_node('position_controller')  # Initializing ros node with name position_controller
        # Initializing Altitude ,Latitude and Longitude values

        self.altitude_coord = 0.0
        self.latitude_coord = 0.0
        self.longitude_coord = 0.0
        #Below, Waypoints holds initially the coordinates of the buildings to visit. The script modifies it further according to necessity.
        self.tempwaypoints = []
        self.waypoints = [[18.9992411381,71.9998195495,18.8,0],[18.9990965925,71.9999050292,22.2,2],[18.9990965928,72.0000664814,10.75,2],[18.9993675932,72.0000569892,10.7,2]]#[[19.0009248718,71.9998318945,24.1965528107,0],[19.0,72.0,24.1965528107,3],[19.0,72.0,8.44,0]]#[[19.0009248718,71.9998318945,24.1965528107,0],[19.0007046575,71.9998955286,24.1965528107,3],[19.0007046575,71.9998955286,22.1599967919,1]]
        self.last_alt = self.waypoints[-1][2]
        self.waypoint = [0,0,0,0]
        self.path_planned = False #Flag to induce move(), the path planner.
        self.in_transit = False #Flag to induce waypoint_setter().

        self.err = [0.0, 0, 0]
        self.origin = [18.9995186026,71.999820774]#[19,72]
        self.setpoint = [3, 19, 72]
        self.setpoints = [] #To hold the output of the path planner.
        # Setting Kp, Ki, Kd values for altitude, latitude, longitude with mentioned indexing order

        self.k_p = [420,5750,5750]
        self.k_i = [0.032,560,560]
        self.k_d = [219,38000,38000]

        # Initializing Error Values for altitude, latitude, longitude with mentioned indexing order

        self.lasterror = [0, 0, 0]
        self.errorsum = [0, 0, 0]
        self.output = [0, 0, 0]
        self.changerror = [0, 0, 0]

        # Declaring edrone_cmd and initialing Values

        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcThrottle = 0.0
        self.drone_cmd.rcRoll = 0.0
        self.drone_cmd.rcPitch = 0.0
        self.drone_cmd.rcYaw = 0.0

        self.marker_data = MarkerData()
        self.marker_data.err_x_m = 0.0
        self.marker_data.err_y_m = 0.0
        self.marker_data.marker_id = 2  #Current building to visit
        
        self.sample_time = 0.06
        self.curr_time = 0
        self.cost = 2 # The cost of travelling between adjecent nodes on the map.
        self.ctr = 0
        self.obstacle_flag = 0 #Indicate if there is an obstacle on the planned route.
        self.target_set = False
        #In the cascade classifier, the full path of the cascade file need to be provided.
        self.cooldown = 0
        self.logo_cascade = cv2.CascadeClassifier()
        self.l = self.logo_cascade.load('/home/sde/Scripts&Projects/Python/Haar_Cascades/intro_cascade_classifiers_training_and_usage/data/cascade.xml')
        self.img = np.empty([])
        self.centre = []
        #Declaring Image subscription
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback)

        # Declaring Publisher to topic drone_command for input to node attitude_controller.py

        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps)
        rospy.Subscriber("/edrone/range_finder_top",LaserScan,self.lidar_callback)

        self.error_x_pub = rospy.Publisher('/edrone/err_x_m',MarkerData,queue_size=1)
        self.error_y_pub = rospy.Publisher('/edrone/err_y_m',MarkerData,queue_size=1)
        self.marker_id_pub = rospy.Publisher('/edrone/curr_marker_id',MarkerData,queue_size=1)

        self.boundaries = boundary_points  # Used to define the region of search.
        self.start = self.relative_mapping(start)
        self.grid = self.make_grid() #Function to make a 2D map of the entire region. This map is dynamically updated by the update() function.
        self.flag = False
        self.to_visit = [(self.start,0,0,0)] # list of node position, f,h,g respectively, for the A* algorithm.
        #self.errors = [0.000004517,0.0000047487,0.2]
        self.visited = set()#{} # Set of visited map nodes for the A* algo.
        self.ranges = [0,0,0,0,0]
    def gps(self, msg):
        """ gps runs each time coordinates are published to topic /edrone/gps """
        self.latitude_coord = msg.latitude
        self.longitude_coord = msg.longitude
        self.altitude_coord = msg.altitude
    def check_err(self,a,f=0): #Checks error from the current passed gps form coordinates.
        alt_err = 0.2
        lat_err = 0.00001#0.000001
        long_err = lat_err
        if f == 1:
            lat_err  = 0.0002 #Try 0.2
            long_err = lat_err
        err = [0,0,0]
        err[0] = a[0] - self.altitude_coord
        err[1] = a[1] - self.latitude_coord
        err[2] = a[2] - self.longitude_coord
        if abs(err[0]) < alt_err and abs(err[1]) < lat_err and abs(err[2]) < long_err:
            return True
        return False
    def lidar_callback(self,msg):
        self.ranges = msg.ranges
    def image_callback(self,data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.loginfo(e)
    def relative_mapping(self,a): #Converts given coordinates to the map coordinates.
        x = abs(110692.0702932625*(a[0] - self.origin[0])/self.cost) #19.0009768383
        y = abs(105292.0089353767*(a[1] - self.origin[1])/self.cost) #71.9998318943
        return (int(x),int(y))
    def make_grid(self):  # Makes a map.
        corners = []
        for boundary in self.boundaries:
            corners.append(self.relative_mapping(boundary))
        self.num_rows = abs(int((corners[2][0] - corners[1][0])/self.cost))
        self.num_cols = abs(int((corners[3][1] - corners[2][1])/self.cost))
        grid = [[self.cost for j in range(self.num_cols)] for i in range(self.num_rows)]
        #rospy.loginfo(grid[18][0])
        return grid
    def update(self): # Used to update the map, as new obstacles are discovered dynamically by the sensor. It basically feeds obstacle location in te map, as -1.
        #self.cost = 1
        self.pos = self.relative_mapping([self.latitude_coord,self.longitude_coord])
        front = self.ranges[3]//self.cost # try ranges[0] here if it does not work.
        if not math.isnan(front):
            if self.pos[0] + front < self.num_rows:
                front = int(front)
                try:
                    self.grid[self.pos[0] + front][self.pos[1]] = -1
                    self.grid[self.pos[0] + front-1][self.pos[1]] = -1
                except Exception as e:
                    pass
                rospy.loginfo("obstacle_front : %s,%s",str(self.pos[0]+front),str(self.pos[1]))
        back = self.ranges[1]//self.cost 
        if not math.isnan(back):
            if self.pos[0] - back >= 0:
                back = int(back)
                try:
                    self.grid[self.pos[0] - back][self.pos[1]] = -1
                    self.grid[self.pos[0]-back+1][self.pos[1]] = -1
                except Exception as e:
                    pass
                rospy.loginfo("obstacle_back : %s,%s",str(self.pos[0]-back),str(self.pos[1]))
        left = self.ranges[2]//self.cost
        if not math.isnan(left):
            if self.pos[1] + left < self.num_cols:
                left = int(left)
                try:
                    self.grid[self.pos[0]][self.pos[1] + left] = -1
                    self.grid[self.pos[0]][self.pos[1]+left-1] = -1
                except Exception as e:
                    pass
                rospy.loginfo("obstacle_left : %s,%s",str(self.pos[0]),str(self.pos[1]+left))
        right = self.ranges[0]//self.cost
        if not math.isnan(right):
            if self.pos[1] - right >= 0:
                right = int(right)
                try:
                    self.grid[self.pos[0]][self.pos[1] - right] = -1
                    self.grid[self.pos[0]][self.pos[1]-right+1] = -1
                except Exception as e:
                    pass
                rospy.loginfo("obstacle_right : %s,%s",str(self.pos[0]),str(self.pos[1]-right))
    def heuristic(self,curr):  # Manhattan heuristic has been used here,(L1)
        return abs(curr[0] - self.end[0]) + abs(curr[1] - self.end[1])
    def detector(self): #Uses the trained Cascade classifier to return the centres of squares found.
        if self.l == False:
            self.l = self.logo_cascade.load('/home/sde/Scripts&Projects/Python/Haar_Cascades/intro_cascade_classifiers_training_and_usage/data/cascade.xml')
        rospy.loginfo(self.l)
        key = cv2.waitKey(1) & 0xFF
        self.img = np.array(self.img,dtype = 'uint8')
        #gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        logo = self.logo_cascade.detectMultiScale(self.img, scaleFactor=1.05,minNeighbors=2)
        centres = []
        for (x, y, w, h) in logo:
            centres.append((int(x + w//2), int(y + h//2)))
            #cv2.rectangle(self.img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        #cv2.imshow("Camera_feed",self.img)
        if key == ord("q"):
            cv2.destroyAllWindows()
        return centres
    def target_setter(self,a): #Based on the centre of square passed, it sets the coordinate to travel to(centre of square)
        rospy.loginfo("###---target_setter_called---###")
        Z_m = self.altitude_coord - self.waypoint[0]
        focal_length = 238.3507189
        dir0 = self.img.shape[0]//2-a[0]
        dir1 = self.img.shape[1]//2-a[1]
        x = dir0*Z_m/focal_length
        y = dir1*Z_m/focal_length
        x/=(-110692.0702932625)
        x+=self.latitude_coord
        y/=(-105292.0089353767)
        y+=self.longitude_coord
        return [x,y]
    def error_in_meters(self): #When the marker coordinates are set, this changes the error in x and y.
        if self.target_set == True:
            try:
                self.marker_data.err_x_m = self.centre[0]*(self.altitude_coord-self.waypoint[0])/238.3507189
                self.marker_data.err_y_m = self.centre[1]*(self.altitude_coord-self.waypoint[0])/238.3507189
            except Exception as e:
                pass
    def marker_status(self): #Publishes current marker being travelled to and the the errors in meters.
        if self.curr_time == 0:
            self.curr_time = time.time()
        else:
            if time.time() - self.curr_time >=1:
                self.error_x_pub.publish(self.marker_data)
                self.error_y_pub.publish(self.marker_data)
                self.marker_id_pub.publish(self.marker_data)
                self.curr_time = time.time()
    def waypoint_setter(self): # Sets the waypoints.
        #each waypoint has a number in the end, which varies from 0 to 3. These determine the mode of operation.
        #0 -> Directly set point.
        #1 -> Need to perform cascade classification, and take action on the basis of that.
        #2 -> Need to reach given point after performing a specified sequence of actions.
        #3 -> Need to reach this waypoint useing application of path planning (A*).

        if self.in_transit == False:
            if len(self.waypoints) != 0:
                self.waypoint[0] = self.waypoints[0][2]
                self.waypoint[1] = self.waypoints[0][0]
                self.waypoint[2] = self.waypoints[0][1]
                self.waypoint[3] = self.waypoints[0][3]
                self.in_transit = True
        rospy.loginfo(self.waypoints)
        if self.waypoint[3] == 0:
            self.setpoint = self.waypoint[:-1]
            if self.check_err(self.setpoint):
                if self.target_set == True:
                    self.target_set = False
                    if self.marker_data.marker_id == 2:
                        self.marker_data.marker_id = 1
                    elif self.marker_data.marker_id == 1:
                        self.marker_data.marker_id = 3
                if self.marker_data.marker_id == 3 and len(self.waypoints) == 0:
                    self.waypoints.append([])
                    #e1,e2 = -3/110692.0702932625,-0.75/105292.0089353767
                    self.waypoints.append([self.waypoint[1],self.waypoint[2],self.last_alt,0])
                try:
                    self.waypoints.pop(0)
                except Exception as e:
                    rospy.loginfo("Task_Complete")
                self.in_transit = False
        if self.waypoint[3] == 2:
            self.waypoints[0][3] = 1
            self.waypoints[0][2] += 1
            if self.waypoint[0] < self.altitude_coord:
                dist = (self.altitude_coord+1-self.waypoint[0])/3
                for i in range(3):
                    self.waypoints = [[self.waypoint[1],self.waypoint[2],self.waypoint[0]+dist,0]]+self.waypoints
                    dist += dist
                #self.waypoints = [[self.waypoint[1],self.waypoint[2],self.waypoint[0]+3,0]]+self.waypoints
                self.waypoints = [[self.waypoint[1],self.waypoint[2],self.altitude_coord+4,0]]+self.waypoints
                self.waypoints = [[self.latitude_coord,self.longitude_coord,self.altitude_coord+4,0]]+self.waypoints
            else:
                self.waypoints = [[self.waypoint[1],self.waypoint[2],self.waypoint[0]+4,0]]+self.waypoints
                self.waypoints = [[self.latitude_coord,self.longitude_coord,self.waypoint[0]+4,0]] + self.waypoints
            self.in_transit = False
        if self.waypoint[3] == 3:
            if self.check_err(self.waypoint[:-1]):
                self.in_transit = False
                self.waypoints.pop(0)
            if self.path_planned == False and self.in_transit == True:
                self.path_planned = True
                self.setpoints = self.move(12,0) #25
                self.setpoints = self.analyse(self.setpoints)
                if len(self.setpoints) > 1:
                    self.setpoints.pop(0)
                rospy.loginfo("Main analysed setpoints above")
                rospy.loginfo(self.setpoints)
                try:
                    x,y = self.inverse_mapping(self.setpoints[0][0],self.setpoints[0][1])
                    self.setpoints.pop(0)
                except Exception as e:
                    pass
                self.setpoint[0],self.setpoint[1],self.setpoint[2] = self.waypoint[0],x,y
            elif self.obstacle_flag == 1 and self.in_transit == True and self.cooldown == 0:
                self.cooldown = 1
                visit = self.to_visit
                self.setpoints = self.move(4,1)#8
                self.to_visit = self.to_visit + visit
                self.setpoints = self.analyse(self.setpoints)
                if len(self.setpoints) > 1:
                    self.setpoints.pop(0)
                rospy.loginfo("Analysed points other than main above")
                rospy.loginfo(self.setpoints)
                try:
                    x,y = self.inverse_mapping(self.setpoints[0][0],self.setpoints[0][1])
                    self.setpoints.pop(0)
                except Exception as e:
                    pass
                self.setpoint[0],self.setpoint[1],self.setpoint[2] = self.waypoint[0],x,y
                #self.called = 0
            if self.cooldown == 1 and self.check_err(self.setpoint,f=1): #Try the regular one here too.
                self.cooldown = 0
            if self.check_err(self.setpoint) and self.in_transit == True:
                if len(self.setpoints) != 0:
                    x,y = self.inverse_mapping(self.setpoints[0][0],self.setpoints[0][1])
                    self.setpoints.pop(0)
                    self.setpoint[0],self.setpoint[1],self.setpoint[2] = self.waypoint[0],x,y
                else:
                    self.path_planned = False
        if self.waypoint[3] == 1:
            self.setpoint = self.waypoint[:-1]
            if self.check_err(self.setpoint):
                rospy.loginfo("Starting detection sequence")
                centres = []
                centres = self.detector()
                if len(centres) == 0:
                    self.ctr += 1
                    if self.ctr >2:
                        self.ctr = 0
                        if len(self.tempwaypoints)>0 and len(self.waypoints)>0:
                            self.waypoints.pop(0)
                            self.in_transit = False
                            return
                        self.tempwaypoints = self.waypoints
                        self.waypoints = []
                        self.waypoints.append([self.latitude_coord,self.longitude_coord,self.altitude_coord+3,0])
                        sign,toapx,toapy,toaph = 1,self.latitude_coord,self.longitude_coord,self.altitude_coord+3
                        iters = 3
                        for a in range(2):
                            for i in range(iters):
                                toapx += sign*2.15/110692.0702932625
                                self.waypoints.append([toapx,toapy,toaph,1])
                            for i in range(iters):
                                toapy += -1*sign*3.31/105292.0089353767
                                self.waypoints.append([toapx,toapy,toaph,1])
                            sign = -1
                            iters = 5
                            toaph += 2
                        self.in_transit = False
                if len(centres) > 0:
                    self.ctr = 0
                    self.centre = centres[0]
                    a = self.target_setter(self.centre)
                    rospy.loginfo(a)
                    if len(self.tempwaypoints) > 0:
                        self.waypoints = self.tempwaypoints
                        self.tempwaypoints = []
                    if len(a) > 0:
                        self.waypoints.pop(0)
                        self.waypoints = [[a[0],a[1],self.altitude_coord,0]] + self.waypoints
                        self.in_transit = False
                        self.target_set = True
    def analyse(self,setpoints): 
        #Takes the setpoints returned by the move() function, and simplifies them to eliminate any points which can be traveled to straight.
        try:
            tempoints = [setpoints[0]]
        except Exception as e:
            pass
        if len(setpoints) >2:
            for i in range(1,len(setpoints)-1):
                curr,prev,nextp = setpoints[i],setpoints[i-1],setpoints[i+1]
                if nextp[0]-curr[0] != curr[0]-prev[0] or nextp[1] - curr[1] != curr[1] - prev[1]:
                    tempoints.append(curr)
            tempoints.append(nextp)
        else:
            tempoints = setpoints
        return tempoints
    def check_obstacle(self):
        #Dynamically checks if there is any obstacle in the immidiate node being travelled to. If yes, then it recalls move() to path plan again.
        self.pos = self.relative_mapping([self.latitude_coord,self.longitude_coord])
        a = self.relative_mapping([self.setpoint[1],self.setpoint[2]])
        x,y = a[0],a[1]
        move_dir = [0,0]
        rospy.loginfo("in x dir: %s",str(x-self.pos[0]))
        rospy.loginfo("in y dir: %s",str(y-self.pos[1]))
        if abs(x-self.pos[0]) < 7 and abs(x- self.pos[1]) < 7:
            self.update()
        try:
            if x == self.pos[0] and y == self.pos[1]:
                self.grid[self.pos[0]][self.pos[1]] = self.cost
            if x-self.pos[0] != 0:
                move_dir[0] = int((x-self.pos[0])//abs(x-self.pos[0]))
            if y-self.pos[1] != 0:
                move_dir[1] = int((y-self.pos[1])//abs(y-self.pos[1]))
        except Exception as e:
            pass
        rospy.loginfo("move_dir above : ")
        rospy.loginfo(move_dir)
        if move_dir == [1,1] or move_dir == [-1,-1]:
            self.obstacle_flag = 1
            return
        try:
            rospy.loginfo("This is position: %s, %s",str(self.pos[0] + move_dir[0]),str(self.pos[1] + move_dir[1]))
            rospy.loginfo("This is obstacle status: %s",str(self.grid[self.pos[0] + move_dir[0]][self.pos[1] + move_dir[1]]))
            if self.grid[self.pos[0] + move_dir[0]][self.pos[1] + move_dir[1]] == -1:
                self.obstacle_flag = 1
            else:
                self.obstacle_flag = 0
        except Exception as e:
            rospy.loginfo(e)
        rospy.loginfo("Obstacle flag: %s",str(self.obstacle_flag))

    def move(self,iters,mode):
        # This uses the A* algo to return a planned path. Runs for specified number of iterations, and relies on dynamic map updates to find obstacle locations.
        self.update()
        if mode == 1:
            self.start = self.relative_mapping([self.latitude_coord,self.longitude_coord])
            self.to_visit = [(self.start,0,0,0)]
        self.end = self.relative_mapping(self.waypoint[1:-1])
        move_order  =  [[-1, 0 ], # go up
                [ 0, -1], # go right
                [ 1, 0 ], # go down
                [ 0, 1 ]] # go left
        planned_points = []
        while len(self.to_visit) > 0 and iters > 0:
            iters -= 1
            rospy.loginfo("planning")
            node_to_visit = self.to_visit[0]
            visit_index = 0
            for index,item in enumerate(self.to_visit): # Node with least f chosen as new node to move to.
                if item[1] < node_to_visit[1]:
                    node_to_visit = item
                    visit_index = index
            self.to_visit.pop(visit_index)
            self.visited.add(node_to_visit[0])
            if node_to_visit[0] == self.end:
                self.to_visit = []
                self.to_visit.append(node_to_visit)
                self.flag = True
                rospy.loginfo("true")
                rospy.loginfo("endlat: %s,endlong: %s,REACHED",str(self.end[0]),str(self.end[1]))
                planned_points.append([self.end[0],self.end[1]])
                break
            neighbours = []
            for order in move_order:
                node_pos = (node_to_visit[0][0] + order[0],node_to_visit[0][1] + order[1])
                if (node_pos[0] >= self.num_rows or  # If this target lies outside the map.
                    node_pos[0] < 0 or 
                    node_pos[1] >= self.num_cols or
                    node_pos[1] < 0):
                    continue
                if self.grid[node_pos[0]][node_pos[1]] == -1:   # -1 represents an obstacle.
                    continue
                neighbours.append([node_pos,0,0,0])
            for n in neighbours:
                if n[0] in self.visited:
                    continue
                n[2] = self.heuristic(n[0])
                n[3] = self.cost
                n[1] = n[3] + n[2]
                if len([i for i in self.to_visit if n[0] == i[0] and n[1] > i[1]]) > 0:  # If node is already in to_visit,and offers better f if approached in some other way.
                    continue
                n = tuple(n)
                self.to_visit.append(n)
            x,y = node_to_visit[0][0],node_to_visit[0][1]
            rospy.loginfo("%s,%s",str(x),str(y))
            planned_points.append([x,y])
        return planned_points

    def pid(self):
        """  Main PID Controller for altitude, latitude, longitude Error """

        # Calculating Error for altitude, latitude, longitude
        self.check_obstacle()
        self.waypoint_setter()
        rospy.loginfo("##Setpoint:%s, %s, %s",str(self.setpoint[1]),str(self.setpoint[2]),str(self.setpoint[0]))
        self.error_in_meters()
        self.marker_status()
        self.err[0] = self.setpoint[0] - self.altitude_coord
        self.err[1] = self.setpoint[1] - self.latitude_coord
        self.err[2] = self.setpoint[2] - self.longitude_coord

        # Calculating Change in Error for altitude, latitude, longitude

        self.changerror[0] = self.err[0] - self.lasterror[0]
        self.changerror[1] = self.err[1] - self.lasterror[1]
        self.changerror[2] = self.err[2] - self.lasterror[2]

        # Calculating sum of Error for altitude, latitude, longitude

        self.errorsum[0] = (self.errorsum[0] + self.err[0])*self.sample_time
        self.errorsum[1] = (self.errorsum[1] + self.err[1])*self.sample_time
        self.errorsum[2] = (self.errorsum[2] + self.err[2])*self.sample_time

        # Calculating Output which is to be sent to attitude_controller.py through edrone/cmd pub

        self.output[0] = self.k_p[0]*self.err[0] + self.k_i[0]*self.errorsum[0] + self.k_d[0]*self.changerror[0]/self.sample_time
        self.output[1] = self.k_p[1]*self.err[1] + self.k_i[1]*self.errorsum[1] + self.k_d[1]*self.changerror[1]/self.sample_time
        self.output[2] = self.k_p[2]*self.err[2] + self.k_i[2]*self.errorsum[2] + self.k_d[2]*self.changerror[2]/self.sample_time

        # Equation for Throttle , Pitch, Roll and Yaw values for attitude_controller.py

        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcThrottle = self.output[0] + 1500
        self.drone_cmd.rcPitch = 1500 + 6*self.output[1]
        self.drone_cmd.rcRoll = 1500 + 6*self.output[2]
        self.drone_cmd.rcYaw = 1500

        # Storing Current error

        self.lasterror[0] = self.err[0]
        self.lasterror[1] = self.err[1]
        self.lasterror[2] = self.err[2]
        # Publishing msg drone_cmd
        self.drone_pub.publish(self.drone_cmd)
if __name__ == '__main__':
    time.sleep(1.5)
    E_DRONE = Edrone(start = [18.9992411381,71.9998195495],boundary_points = [[18.9995186026,71.999820774],[18.9995186026,72.0001410349],[18.998997735,72.0001410349],[18.998997735,71.9997944347]])#start = [19.0009248718,71.9998318945],boundary_points = [[19.0009768383,71.9998318943],[19.0009768383,72.0001281995],[18.9997277318,72.0001281995],[18.9997277318,71.9998318943]])
    R = rospy.Rate(16.67)
    while not rospy.is_shutdown():
        E_DRONE.pid()
        R.sleep()