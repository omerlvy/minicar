# Author: Christian Careaga (christian.careaga7@gmail.com)
# A* Pathfinding in Python (2.7)
# Please give credit if used

import numpy
import rospy
from heapq import *

from math import atan2,pi,cos,sin
radius = 5
epsilon = 15
criticalAngle = pi/5
errorDegFactor = 2
#heuristic function
def heuristic(a, b):
    errorDeg = atan2(a[1]-b[1],a[0]-b[0])
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)**1/2 + errorDegFactor * abs(errorDeg*2*radius/pi)


def astar(array, goal, start):
    print goal, "goal"
    print start, "start"
    close_set = set()
    came_from = {} #like Dijakstra, by the end we will collect all of them in order to generate the path
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start)) 

    while oheap: #run until the heap is not empty

        current = heappop(oheap)[1]

        if (abs(current[0]-goal[0])<epsilon and abs(current[1]-goal[1])<epsilon): #we can't actually get to the specified point, so we have defined a converagnce range, this is a break point
            data = []
            while current in came_from: #collect the fathers
                data.append(current)
                current = came_from[current]
            data.append(start)
            return data #break

        close_set.add(current)
        ''' choose right neighbors'''
        deg = (((current[2] % (2*pi)) / pi) * 180) % 360 #current angle
        neighbors =[]
        optionalDegPlus = ((((current[2] + criticalAngle) % (2*pi)) / pi) * 180) % 360 #possible angle in the positive direction
        optionalDegMimus = ((((current[2] - criticalAngle) % (2 * pi)) / pi) * 180) % 360 #possible angle in the negative direction
	#choose the neighbors by the possible angles
        if (deg>=45*0 and deg<26.56):
            neighbors = [(1,0),(2,1)]
        if (deg>=26.57 and deg<45):
            neighbors = [(2,1),(1,1)]
        elif (deg>=45*1 and deg<63.43):
            neighbors = [(1, 1), (1, 2)]
        if (deg>=63.43 and deg<90):
            neighbors = [(1, 2),(0,1)]
        elif (deg>=45*2 and deg<116.56):
            neighbors = [(0, 1), (-1, 2)]
        elif (deg>=116.56 and deg<135):
            neighbors = [(-1, 2), (-1, 1)]
        elif (deg>=135 and deg<153.43):
            neighbors = [(-1, 1), (-2, 1)]
        elif (deg>=153.43 and deg<180):
            neighbors = [(-2, 1), (-1, 0)]
        elif (deg>=180 and deg<206.56):
            neighbors = [(-1, 0), (-2, -1)]
        elif (deg>=206.56 and deg<225):
            neighbors = [(-2, -1), (-1, -1)]
        elif (deg>=225 and deg<243.43):
            neighbors = [(-1, -1), (-1, -2)]
        elif (deg>=243.43 and deg<270):
            neighbors = [(-1, -2), (0, -1)]
        elif (deg>=270 and deg<296.56):
            neighbors = [(0, -1), (1, -2)]
        elif (deg>=296.56 and deg<2315):
            neighbors = [(1, -2), (1, -1)]
        elif (deg>=315 and deg<333.43):
            neighbors = [(1, -1), (2, -1)]
        elif (deg>=333.43 and deg<360):
            neighbors = [(2, -1), (1, 0)]
        for i, j in neighbors:
            isOut = 0
            count = 0
#every neighbor keeps his angle, so we update this field, with modolu function
            if ((current[2] % 2*pi) - (atan2(j,i) % 2*pi)>0): #modolu (more than 360)
                if (current[2] - criticalAngle < -1*pi):#modolu
                    neighbor = current[0] + i, current[1] + j, current[2] - criticalAngle + 2*pi #x, y, angle
                else:
                    neighbor = current[0] + i, current[1] + j, current[2] - criticalAngle
            else:
                if (current[2] + criticalAngle > pi):#modolu
                    neighbor = current[0] + i, current[1] + j, current[2] + criticalAngle - 2*pi
                else:
                    neighbor = current[0] + i, current[1] + j, current[2] + criticalAngle
            tentative_g_score = gscore[current] + heuristic(current, neighbor) #calculate with the heuristic function
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    length = 0.5 #[m]
                    width = 0.35  #[m]
                    normalLength = int(length / 0.05)
                    normalWidth = int(width / 0.05)
                    for i in range(-normalLength, 1):
                        for j in range((-normalWidth / 2)  , (normalWidth / 2) + 1):
                            if (i== 0 or j== normalWidth/2 or i== -normalLength or j == -normalWidth/2):
                                degree = neighbor[2] #the degree field of the neighbor
                                count=count+1
                                if(int(array[int(i * cos(degree) - j * sin(degree))+ int(neighbor[0])][(int(i * sin(degree) + j * cos(degree)))+ int(neighbor[1])]==1)):
                                    isOut = 1
                                    break
                            if isOut == 1:
                                break
                        if isOut == 1:
                            continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
#A* generic instructions, updating next positions, etc
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if int(array[int(neighbor[0])][int(neighbor[1])])!=1:
                    heappush(oheap, (fscore[neighbor], neighbor))

    return False


'''Here is an example of using my algo with a numpy array,
   astar(array, start, destination)
   astar function returns a list of points (shortest path)

nmap = numpy.array([
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        neighbors =[]

print astar(nmap, (0, 0), (10, 13))
[(10, 13), (9, 12), (8, 11), (8, 10), (8, 9), (8, 8), (8, 7), (8, 6), (8, 5), (8, 4), (8, 3), (8, 2), (7, 1), (6, 2), (6, 3), (6, 4), (6, 5), (6, 6), (6, 7), (6, 8), (6, 9), (6, 10), (6, 11), (5, 12), (4, 11), (4, 10), (4, 9), (4, 8), (4, 7), (4, 6), (4, 5), (4, 4), (4, 3), (4, 2), (3, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6), (2, 7), (2, 8), (2, 9), (2, 10), (2, 11), (1, 12), (0, 11), (0, 10), (0, 9), (0, 8), (0, 7), (0, 6), (0, 5), (0, 4), (0, 3), (0, 2), (0, 1)]
'''
