#!/usr/bin/env python
import numpy
from tf.transformations import euler_from_quaternion
import pickle
from math import sqrt, sin, cos
import rospy
import cv_bridge
import cv2
from geometry_msgs.msg import PoseStamped, Point, Pose, TransformStamped, PolygonStamped, Point32, _Point32, Polygon, Twist
from nav_msgs.msg import OccupancyGrid,Odometry,Path
import matplotlib.pyplot as plt
#import  Astar
import AstarDeg
from math import pi
import geometry_msgs
from tf.msg import tfMessage
from math import atan2
import tf
from scipy.interpolate import interp1d
from scipy import interpolate
class AStarSearch:
    MIN_DISTANCE = 10
    def __init__(self):
        self.goal_ = PoseStamped()
        self.odom_ = Odometry()
        self.a_star_pub = rospy.Publisher('/a_star/path', Path, queue_size=1)
        rospy.init_node('a_star', anonymous=True)

        #subscribers
        rospy.Subscriber("/rtabmap/grid_map", OccupancyGrid, self.OccupancyGrid_callback) #the map fromrtabmap (you can also use other maps)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback) #the destination from rviz
        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback)
        #print "Odometry Points"
	#print Odometry #************* print the front of the wheel
        rospy.Subscriber("/tf", tfMessage, self.tf_callbak) #position

    #goal subscriber goal from rviz
    def goal_callback(self, data=PoseStamped()):
        self.goal_ = data #updating goal

    def tf_callbak(self, data=tfMessage()):
        obstacle_msg = PolygonStamped()
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = "map"  # CHANGE HERE: odom/map
        for i in data.transforms:
            if i.child_frame_id == "camera_link":
                tempPose = PoseStamped()
                tempPose.pose.position.x = -1*i.transform.translation.x #axis reverse
                tempPose.pose.position.y = i.transform.translation.y
                tempPose.pose.position.z = i.transform.translation.z
                x = i.transform.rotation.x
                y = i.transform.rotation.y
                z = i.transform.rotation.z
                w = i.transform.rotation.w
                quaternion = (x,y,z,w)
                self.initialAngle = tf.transformations.euler_from_quaternion(quaternion)[2] #euler to radians
                self.camera_link = tempPose
                try:
                    self.myCord = self.vectortoCord(self.camera_link, self.org.position.x, self.org.position.y, 0.05)
                except AttributeError:
                    print "catch"
                obstacle_msg.polygon.points = []
		#painting the polygon
                length = 50 #cm vehicle dim
                width = 35 #cm vehicle dim
		#the origin point is at the head of the car (y axis), and in the midlle (x axis)
                for i in range (-length, 0): 
                    for j in range (-width/2, width/2):
                        tempPointOrien = Point32()
                        degree = self.initialAngle
			#orientation with multiplication by transform matrix
                        tempPointOrien.x = (i * 0.01 * cos(degree) - j * 0.01 * sin(degree)) - tempPose.pose.position.x
                        tempPointOrien.y = (i * 0.01 * sin(degree) + j * 0.01 * cos(degree)) + tempPose.pose.position.y
                        tempPointOrien.z = 0
                        obstacle_msg.polygon.points.append(tempPointOrien)
                pub2 = rospy.Publisher('/geometry_msgs/Polygon', PolygonStamped, queue_size=10)
                pub2.publish(obstacle_msg) #publish msg
    def odom_callback(self, msg=Odometry()):
        self.odom_ = msg

    def goal_callback(self, msg=PoseStamped):
        self.goal_ = msg
        #print msg #updating goal
        tempPose = PoseStamped()
        tempPose.pose.position.x = -1*msg.pose.position.x
        tempPose.pose.position.y = msg.pose.position.y
        self.stampedCord = self.vectortoCord(tempPose, self.org.position.x, self.org.position.y, 0.05)
        #print "Stamped cord", self.stampedCord
        #print "my cord", self.myCord

    def OccupancyGrid_callback(self, data=OccupancyGrid()):
        temp_data =[]
	#this loop transform the probability vector to boolean vector. We havn't seen even one time values differ from 0,100
        for i in data.data:
            temp_data.append(0 if (i==-1 or i==0) else 1)
        yyyy = temp_data
        data.data=temp_data
        self.org = data.info.origin
        path = Path()
        path.header = data.header
        if self.goal_.header.seq == 0:
            print "path"
            return path
        ''' A* loop, we haben't really used this pattern, but it is recomendded '''
        self.resolution = data.info.resolution
        temp_mat = []
        i = 0
	#make the vector (1D array) to 2D array
        width = data.info.width
        while (i < len(data.data)):
            temp_mat.append(data.data[i:i + width])
            i = i + width
	#transpose for moving from 3D XYZ to 2D XY
        matrix = numpy.array(numpy.transpose(temp_mat))

        obstaclesVector = []
        counter = 0
	#infalting the obstacles, making every obstable bigger, we did it staticly (the number 10 below), if you can, do it dynamiclly (orientation)
	#two loops, the first one saves the obstacles and the second inflates
        for x in range(0 , len(matrix)-1):
            for y in range(0 , len(matrix[0])-1):
                if matrix[x][y] == 1:
                    counter = counter+110 
                    obstaclesVector.append((x,y))
        for xObs , yObs in obstaclesVector:
            for i in range(0,10): #static
                for j in range(0, 10):
                    try:
                        matrix[xObs+i][yObs+j] = 1
                    except IndexError:
                        pass
	#writing to file if you find it helpful
        '''file = open('testfile.txt', 'w')
        for i in temp_mat:
            for j in i:
                file.write(str(j))
            file.write('\n')
            file.close()
        '''

        astarpath = AstarDeg.astar(matrix,(self.stampedCord[0],self.stampedCord[1],0), (self.myCord[0], self.myCord[1],self.initialAngle)) #calling the A* function, you can find in the file named - 
	#print "This is the Astar path BEFORE interpolation"        
	#print(astarpath)
        ''' interpolate, take a look in the final report, it is not an easy interpolation (not 2D)'''
        xVector = [];#debugging
        yVector = [];#debugging
        L = []; #indexing
        for Cord in astarpath:
            xVector.append(Cord[0]) #debugging
            yVector.append(Cord[1]), "22222", Cord[1]#debugging
            L.append((Cord[0], Cord[1]))
        X = numpy.array(L).T
        t = range(0, len(L)) #the function is below, python doesn't have function for this
        f = interpolate.interp1d(t,X) #generating the interpolation function
        astarpathbackup = astarpath
        astarpath = []
	#make every point in the A* path to the interpolaited point
        for index in numpy.arange(t[0], len(t)-1, 0.1):
            flat = f(index).flat
            flat1D = flat.copy()
            astarpath.append((flat1D[0],flat1D[1]))
        xnew = numpy.linspace(min(xVector), max(xVector), num=len(astarpath) * 5, endpoint=True)
        ''' end interpolate'''
        #astarpath = AstarDeg.astar(matrix, (self.stampedCord[0], self.stampedCord[1], 0),(self.myCord[0], self.myCord[1], pi/3))
	#print "This is the Astar path after interpolation"        
	#print astarpath
	#sending back the resolution
        tempPath = Path()
        tempPath.header = data.header
        tempPath.header.frame_id = "map"

        for i in astarpath:
            temp_point = Point()
            temp_point.x = (i[0] * 0.05) + self.org.position.x
            temp_point.y = (i[1] * 0.05)  + self.org.position.y
            temp_point.z = 0
            tempPose = Pose()
            tempPose.position = temp_point
            tempPose.orientation.x = 0
            tempPose.orientation.y = 0
            tempPose.orientation.z = 0
            tempPose.orientation.w = 1
            tempPoseStamped = PoseStamped()
            tempPoseStamped.header = data.header
            tempPoseStamped.header.frame_id = "map"
            tempPoseStamped.pose = tempPose
            tempPath.poses.append(tempPoseStamped)

        '''movement control and driving instruction for embedded'''
        twist = Twist()
        current = tempPath.poses[0].pose
        desire = tempPath.poses[4].pose
	
	print "desire.position.y=",(desire.position.y) "desire.position.x=",(desire.position.x) 
        angle_to_goal = atan2(desire.position.y - current.position.y, desire.position.x - current.position.x) #the attitude angle to the desire pose
        (pitch, roll, current_angle) = euler_from_quaternion([current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w])
        diffAngle = current_angle-angle_to_goal
	print "diffAngle=" ,(diffAngle), "current_angle=" ,(current_angle), "angle_to_goal" ,(angle_to_goal)
        twist.linear.x = 0.2#200 ###########################################################################
        twist.angular.z = diffAngle*250/(pi)############################################################
        '''movement control and driving instruction for embedded -- end'''

        pub = rospy.Publisher('/nav_msgs/Path', Path, queue_size=10) # the path to the destination
        pubTwist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
      #  rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
       # while not rospy.is_shutdown():
            #hello_str = "hello world %s" % rospy.get_time()
            #rospy.loginfo(hello_str)
        pub.publish(tempPath)
        pubTwist.publish(twist)
        #    rate.sleep()

	
    @staticmethod
    def calc_distance(pose1=PoseStamped(), pose2=PoseStamped()):
        pose1 = pose1.pose.position
        pose2 = pose2.pose.position
        distance = sqrt((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)
        return distance

    @staticmethod
    #given the vector from out current location to the (0,0), calculate the cordinates of our current location
    def vectortoCord(pose1=PoseStamped(), orgX=0, orgY= 0, res=0.05):
        pose1 = pose1.pose.position
        return (round(-1*(pose1.x + orgX) / res)) , round((-1*(orgY - pose1.y) / res))


    def spin(self): #inportatant!!!! might causes communication problem
        rospy.spin()
def frange(start, stop, step):
     i = start
     while i < stop:
         yield i
         i += step
if __name__ == '__main__':
        astar = AStarSearch()
        astar.spin()
