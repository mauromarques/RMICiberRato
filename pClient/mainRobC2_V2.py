import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import os
import time
import math
import heapq

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.directions = []
        self.gpsLineData = {}
        self.mappingMemory_exploration = {}
        self.mappingMemory_drawing = {}
        self.mappingMemory_visitedPoints = {}
        self.mappingMemory_acessPoints ={}
        self.instructionsToFollow = []
        self.lineMemory = []
        self.shouldTurn = False
        self.lastEvenLocation = (0,0)

        self.currentLocation = (0,0)
        self.previousLocation = (-2,-2)
        self.distance = (0,0)
        self.localCurves = {}

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    global pointIntial
    pointIntial = []
    global data
    
    def grid_coordinates(self): 
        location = self.currentLocation
        curves = [x for (Clocation, x), item in self.mappingMemory_exploration.items() if Clocation == location]
        print(location)
        print(curves)
        print("--------")
        self.mappingMemory_visitedPoints[location] = True
        #Create array
        arrayWithAngle = []
        for curve in curves:
            if curve == 90:
                self.mappingMemory_drawing[(location[0], location[1]+1)] = "|"
                arrayWithAngle.append(((location[0], location[1]+2),curve))
                if (location[0], location[1]+2) not in self.mappingMemory_visitedPoints:
                    self.mappingMemory_visitedPoints[(location[0], location[1]+2)] = False
                    self.mappingMemory_acessPoints[(location[0], location[1]+2)] = [(location, -90)]
            if curve == -90:
                self.mappingMemory_drawing[(location[0], location[1]-1)] = "|"
                arrayWithAngle.append(((location[0], location[1]-2),curve))
                if ((location[0], location[1]-2)) not in self.mappingMemory_visitedPoints:
                    self.mappingMemory_visitedPoints[(location[0], location[1]-2)] = False
                    self.mappingMemory_acessPoints[(location[0], location[1]-2)] = [(location, 90)]
            if curve == 180 or curve == -180:
                self.mappingMemory_drawing[(location[0]-1, location[1])] = "-"
                arrayWithAngle.append(((location[0]-2, location[1]),curve))
                if ((location[0]-2, location[1])) not in self.mappingMemory_visitedPoints:
                    self.mappingMemory_visitedPoints[(location[0]-2, location[1])] = False
                    self.mappingMemory_acessPoints[(location[0]-2, location[1])] = [(location, 0)]
            if curve == 0:
                self.mappingMemory_drawing[(location[0]+1, location[1])] = "-"
                arrayWithAngle.append(((location[0]+2, location[1]),curve))
                if ((location[0]+2, location[1])) not in self.mappingMemory_visitedPoints:
                    self.mappingMemory_visitedPoints[(location[0]+2, location[1])] = False
                    self.mappingMemory_acessPoints[(location[0]+2, location[1])] = [(location, 180)]
            if curve == 135:
                self.mappingMemory_drawing[(location[0]-1, location[1]+1)] = "\\"
                arrayWithAngle.append(((location[0]-2, location[1]+2),curve))
                if ((location[0]-2, location[1]+2)) not in self.mappingMemory_visitedPoints:
                    self.mappingMemory_visitedPoints[(location[0]-2, location[1]+2)] = False
                    self.mappingMemory_acessPoints[(location[0]-2, location[1]+2)] = [(location, -45)]
            if curve == -45:
                self.mappingMemory_drawing[(location[0]+1, location[1]-1)] = "\\"
                arrayWithAngle.append(((location[0]+2, location[1]-2),curve))
                if ((location[0]+2, location[1]-2)) not in self.mappingMemory_visitedPoints:
                    self.mappingMemory_visitedPoints[(location[0]+2, location[1]-2)] = False
                    self.mappingMemory_acessPoints[(location[0]+2, location[1]-2)] = [(location, 135)]
            if curve == -135:
                self.mappingMemory_drawing[(location[0]-1, location[1]-1)] = "/"
                arrayWithAngle.append(((location[0]-2, location[1]-2),curve))
                if ((location[0]-2, location[1]-2)) not in self.mappingMemory_visitedPoints:
                    self.mappingMemory_visitedPoints[(location[0]-2, location[1]-2)] = False
                    self.mappingMemory_acessPoints[(location[0]-2, location[1]-2)] = [(location,45)]
            if curve == 45:
                self.mappingMemory_drawing[(location[0]+1, location[1]+1)] = "/"
                arrayWithAngle.append(((location[0]+2, location[1]+2),curve))
                if ((location[0]+2, location[1]+2)) not in self.mappingMemory_visitedPoints:
                    self.mappingMemory_visitedPoints[(location[0]+2, location[1]+2)] = False
                    self.mappingMemory_acessPoints[(location[0]+2, location[1]+2)] = [(location, -135)]
        #Add array to dictionary using location
        self.mappingMemory_acessPoints[location] = arrayWithAngle
        self.writeMapFile()

    def a_star_search(self, start, goal):
        print("ASTAR------")
        priority_queue = [(0, start, [])]
        visited = set()
        while priority_queue:
            cost, current, path = heapq.heappop(priority_queue)
            if current not in visited:
                visited.add(current)
                if current == goal:
                    return path
                print("CURRENT: " + str(current))
                for (neighbor, angle) in self.mappingMemory_acessPoints.get(current):
                    print("Neighbor: " + str(neighbor) + "  " + str(angle))
                    if neighbor not in visited:
                        new_cost = cost + 1
                        heapq.heappush(priority_queue, (new_cost, neighbor, path + [(current, angle)]))
        return None  # No path found

    def writeMapFile(self):
        line_content = " " * 49 
        lines = [line_content] * 21
        file_content = '\n'.join(lines)

        for line_index, line in enumerate(file_content.split('\n')):
            for char_index, char in enumerate(line):
                address = (char_index + 1 - 25, 11 - line_index - 1)
                if address in self.mappingMemory_drawing:
                    lines[line_index] = lines[line_index][:char_index] + self.mappingMemory_drawing[address] + lines[line_index][char_index + 1:]
                    file_content = '\n'.join(lines)

        with open('mapFile.txt', 'w') as file:
            file.write(file_content)


    def correct_motor_speeds(self, left_speed, right_speed):
        compass_angle = self.measures.compass
        angles = [0, 45, 90, 135, 180, -135, -90, -45, -180]
        angle_diffs = [abs(angle - compass_angle) for angle in angles]
        min_index = angle_diffs.index(min(angle_diffs))
        desired_angle = angles[min_index]
        angle_error = desired_angle - compass_angle
        correction_factor = abs(angle_error) / 45
        if angle_error > 0:
            left_speed *= (1 - correction_factor)
            right_speed *= (1 + correction_factor)
        else:
            left_speed *= (1 + correction_factor)
            right_speed *= (1 - correction_factor)
        return(left_speed, right_speed)


    def move(self, line_sensor_values):
        # Example line_sensor_values: [0, 0, 1, 1, 1, 1, 0]

        # Define speed values for left and right wheels
        left_wheel_speed = 0
        right_wheel_speed = 0

        # Proportional control factor
        k_p = 0.05
        base_speed = 0.11
        if abs(self.getAngle()) == 135 or abs(self.getAngle()) == 45 :
            if self.distance > 1.7:
                base_speed = 0.015
            if self.distance > 2.5:
                base_speed = 0.01
        else:
            if self.distance > 1:
                base_speed = 0.015
            if self.distance > 1.8:
                base_speed = 0.01

        # Check if all center sensors are on the line
        if line_sensor_values == ["0","0","1","1","1","0","0"]:
            # All center sensors on the line, move forward
            # removes the oldest item in the history of directions
            if len(self.directions) >= 3:
                self.directions.pop(0)
            self.directions.append("F")
            left_wheel_speed = base_speed
            right_wheel_speed = base_speed
            #print("--------------------")
            #print(self.directions)
            #print(line_sensor_values)
            #print("--------------------")
        else:
            try:
                # Calculate turning adjustment based on the distribution of line sensors
                line_sensor_values = [int(x) for x in line_sensor_values]
                left_sum = line_sensor_values[2]
                right_sum = line_sensor_values[4]

                # Adjust speeds based on the distribution of line sensors
                left_wheel_speed = base_speed + k_p * (right_sum - left_sum)
                right_wheel_speed = base_speed + k_p * (left_sum - right_sum)

                # Cap the speeds to the maximum value
                left_wheel_speed = max(0, min(left_wheel_speed, 0.15))
                right_wheel_speed = max(0, min(right_wheel_speed, 0.15))
            except ValueError:
                # '1' not present in the array, set default speeds
                left_wheel_speed = -base_speed
                right_wheel_speed = base_speed
                # removes the oldest item in the history of directions
                if len(self.directions) >= 3:
                    self.directions.pop(0)

        # Call driveMotors function with calculated speeds
        corrected_speeds = self.correct_motor_speeds(left_wheel_speed, right_wheel_speed)
        self.driveMotors(corrected_speeds[0], corrected_speeds[1])
        if abs(self.getAngle()) == 135 or abs(self.getAngle()) == 45 :
            if self.distance > 2.3:
                corrected_speeds = self.correct_motor_speeds(base_speed, base_speed)
                self.driveMotors(corrected_speeds[0], corrected_speeds[1])
        else:
            if self.distance > 1.5:
                corrected_speeds = self.correct_motor_speeds(base_speed, base_speed)
                self.driveMotors(corrected_speeds[0], corrected_speeds[1])

    def is_at_least_two_away(cself, current_location, previous_location):
        return any(abs(curr - prev) >= 2 for curr, prev in zip(current_location, previous_location))

    def updatePaths(self):
        RBits = ""
        LBits = ""
        for i in range(len(self.lineMemory)):
            if i > 0:
                previousLine = self.lineMemory[i-1]
            else:
                previousLine = None
            if i < len(self.lineMemory)-1:
                nextLine = self.lineMemory[i+1]
            else:
                nextLine = None
            line = self.lineMemory[i]
            
            if (nextLine != None and (nextLine[0] + nextLine[1] == line[0]+line[1])) or (previousLine != None and (previousLine[0] + previousLine[1] == line[0]+line[1])):
                if len(RBits) == 0 or line[0]+line[1] != RBits[-2]+RBits[-1]:
                    RBits = RBits + line[0]+line[1]
            if (nextLine != None and ( nextLine[5] + nextLine[6] == line[5]+line[6])) or (previousLine != None and (previousLine[5] + previousLine[6] == line[5]+line[6])):
                if len(LBits) == 0 or line[5]+line[6] != LBits[-2]+LBits[-1]:
                    LBits = LBits + line[5]+line[6]
                print(line)
        print("-----")
        print("RBits: " + RBits)
        print("LBits: " + LBits)

        r_sequences_found = set()
        l_sequences_found = set()

        # Define the transitions for the finite state machine
        transitions = {
            "start": {"0": "0", "1": "1"},
            "0": {"0": "00", "1": "01"},
            "00": {"0": "0", "1": "1"},
            "01": {"0": "10", "1": "011"},
            "011": {"0": "10", "1": "0111"},
            "0111": {"0": "01110", "1": "01111"},
            "01110": {"0": "n135_n90", "1": "135"},
            "n135_n90": {"0": "0", "1": "1"},
            "01111": {"0": "45", "1": "11"},
            "45": {"0":"0", "1":"1"},
            "1": {"0": "10", "1": "90"},
            "90": {"0": "110", "1": "111"},
            "111": {"0": "45", "1": "90"},
            "110": {"0": "0", "1": "1101"},
            "1101": {"0": "11010", "1": "1"},
            "11010": {"0": "135", "1": "01"},
            "10": {"0": "0", "1": "101"},
            "101": {"0": "10", "1": "1011"},
            "1011": {"0": "10110", "1": "0111"},
            "10110": {"0": "135_90", "1": "135"},
            "135_90": {"0": "0", "1": "1"},
            "135": {"0": "0", "1": "1"},
        }

        state = "start"
        if "0010111000" in RBits or "00101101111000" in RBits:
            r_sequences_found.add("135")
            r_sequences_found.add("90")
            r_sequences_found.add("45")
        for bit in RBits:
            state = transitions[state][bit]
            # Check if a sequence is completed
            if state == "45":
                r_sequences_found.add("45")
            elif state == "90":
                r_sequences_found.add("90")
            elif state == "135":
                r_sequences_found.add("135")
            elif state == "135_90":
                r_sequences_found.add("45")
                r_sequences_found.remove("45")
                r_sequences_found.add("135")
                r_sequences_found.add("90")
        state = "start"
        if "0001110100" in LBits or "00011110110100" in LBits:
            l_sequences_found.add("-135")
            l_sequences_found.add("-90")
            l_sequences_found.add("-45")
        for bit in LBits:
            state = transitions[state][bit]
            # Check if a sequence is completed
            if state == "45":
                l_sequences_found.add("-135")
            elif state == "90":
                l_sequences_found.add("-90")
            elif state == "135":
                l_sequences_found.add("-45")
            elif state == "n135_n90":
                l_sequences_found.add("-45")
                l_sequences_found.remove("-45")
                l_sequences_found.add("-135")
                l_sequences_found.add("-90")
        
        angle = self.getAngle()
        print()
        
        for curve in r_sequences_found:
            curveInt = int(curve)
            if abs(curveInt + angle) <= 180:
                curveInt = curveInt + angle
            else:
                curveInt = angle - curveInt
            if angle == 180 :
                curveInt = curveInt * -1
            if (self.currentLocation, curveInt) not in self.mappingMemory_exploration:
                self.mappingMemory_exploration[(self.currentLocation, curveInt)] = False
                self.localCurves[(self.currentLocation, curveInt)] = int(curve)

        for curve in l_sequences_found:
            curveInt = int(curve)
            if abs(curveInt + angle) <= 180:
                curveInt = curveInt + angle
            else:
                curveInt = angle - curveInt

            if (self.currentLocation, curveInt) not in self.mappingMemory_exploration:
                self.mappingMemory_exploration[(self.currentLocation, curveInt)] = False
                self.localCurves[(self.currentLocation, curveInt)] = int(curve)
        curveInt = 180
        if abs(curveInt + angle) <= 180:
            curveInt = curveInt + angle
        else:
            curveInt = angle - curveInt
        if self.lineMemory[-1][3] == "1":
            if (self.currentLocation, angle) not in self.mappingMemory_exploration:
                self.mappingMemory_exploration[(self.currentLocation, angle)] = False
                self.localCurves[(self.currentLocation, angle)] = 0
        self.mappingMemory_exploration[(self.currentLocation, curveInt)] = True
        self.localCurves[(self.currentLocation, curveInt)] = 180
        self.lineMemory = []

    def getAngle(self):
        if abs(self.measures.compass) <= 180 and abs(self.measures.compass) > 157.5:
            return 180
        if abs(self.measures.compass) >= 0 and abs(self.measures.compass) < 22.5:
            return 0
        elif self.measures.compass >= 22.5 and self.measures.compass < 67.5:
            return 45
        elif self.measures.compass > 22.5-180 and self.measures.compass <= 67.5-180:
            return -135
        elif self.measures.compass >= 67.5 and self.measures.compass < 112.5:
            return 90
        elif self.measures.compass <= -67.5 and self.measures.compass > -112.5:
            return -90
        elif self.measures.compass >= 112.5 and self.measures.compass < 157.5:
            return 135
        elif self.measures.compass > 112.5-180 and self.measures.compass <= 157.5-180:
            return -45
    
    def isCentralLine(self):
        if (self.measures.lineSensor[2] == "1" and self.measures.lineSensor[3] == "1") or (self.measures.lineSensor[3] == "1" and self.measures.lineSensor[4] == "1"):
            return True
        else:
            return False

    def turnTo(self, angle):
        localCurve = self.localCurves[(self.currentLocation, angle)]
        print("Turning to:")
        if angle == -180:
            angle = angle * -1
        print(angle)
        while self.getAngle() != angle or self.isCentralLine() != True:
            base_speed = 0.05
            if localCurve < 0:
                self.driveMotors(base_speed, -base_speed)
            else:
                self.driveMotors(-base_speed, base_speed)
            self.readSensors()
        return
    
    def get_unvisited_angles(self):
        unvisited_angles = []

        if self.currentLocation in self.mappingMemory_acessPoints:
            for (point, angle) in self.mappingMemory_acessPoints[self.currentLocation]:
                if point not in self.mappingMemory_visitedPoints or not self.mappingMemory_visitedPoints[point]:
                    unvisited_angles.append(angle)

        return unvisited_angles

    def distance2points(self,point1, point2):
        return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

    def find_closest_false_point(self ):
        priority_queue = [(self.distance2points(self.currentLocation, point), point) for point, visited in self.mappingMemory_visitedPoints.items() if not visited]
        if not priority_queue:
            return None  # No unvisited points found
        _, closest_point = heapq.nsmallest(1, priority_queue)[0]
        return closest_point    

    def choosePath(self):
        unvisitedAngles = self.get_unvisited_angles()
        #Decidir o goal para o A*
        try:
            if len(unvisitedAngles) == 0:
                goal = self.find_closest_false_point()
                print("GOAL: "+str(goal))
                self.instructionsToFollow = self.a_star_search(self.currentLocation, goal)
                print(self.instructionsToFollow)
                angle = self.instructionsToFollow.pop(0)
                print("ANGLE TO TURN FROM INSTRUCTIONS: " + str(angle))
                self.turnTo(angle[1])
            else:
                self.turnTo(unvisitedAngles[0])
        except:
            self.finish()

        """print(self.instructionsToFollow)
        if len(self.instructionsToFollow) == 0:
            goal = self.find_closest_false_point()
            print("GOAL: "+str(goal))
            self.instructionsToFollow = self.a_star_search(self.currentLocation, goal)
            print(self.instructionsToFollow)
            angle = self.instructionsToFollow.pop(0)
            print("ANGLE TO TURN FROM INSTRUCTIONS: " + str(angle))
            self.turnTo(angle[1])
        else:
            angle = self.instructionsToFollow.pop(0)
            print("ANGLE TO TURN FROM INSTRUCTIONS: " + str(angle))
            self.turnTo(angle[1])"""

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        #state = 'stop'
        #stopped_state = 'run'

        global initialGPS, count, intermediateGPS
        initialGPS = (0.0,0.0)
        intermediateGPS = (0.0,0.0)
        count = 0

        while True:
            self.readSensors()
            line = self.measures.lineSensor
          
            if initialGPS == (0.0,0.0):
                initialGPS = (self.measures.x, self.measures.y)
                intermediateGPS = (self.measures.x, self.measures.y)
            self.currentLocation = (round(self.measures.x - initialGPS[0]), round(self.measures.y - initialGPS[1]))
            
            gpsx = self.measures.x - intermediateGPS[0]
            gpsy = self.measures.y - intermediateGPS[1]
            self.distance = math.sqrt(gpsx**2 + gpsy**2)
            if abs(self.getAngle()) == 135 or abs(self.getAngle()) == 45 :
                if self.distance >= 2.828 and self.shouldTurn == True: 
                    intermediateGPS = (self.measures.x, self.measures.y)
                    self.lastEvenLocation = self.currentLocation
                    self.shouldTurn = False
                    self.driveMotors(0,0)
                    self.updatePaths()  
                    self.grid_coordinates()
                    self.choosePath()
                else:
                    if self.distance > 0.5:
                        self.shouldTurn = True
                        self.lineMemory.append(line)
                    self.move(line)
                    #self.driveMotors(0.1, 0.1)
            else:
                if self.distance >= 2 and self.shouldTurn == True: 
                    intermediateGPS = (self.measures.x, self.measures.y)
                    self.lastEvenLocation = self.currentLocation
                    self.shouldTurn = False
                    self.driveMotors(0,0)
                    self.updatePaths()  
                    self.grid_coordinates()
                    self.choosePath()
                else:
                    if self.distance > 0.5:
                        self.shouldTurn = True
                        self.lineMemory.append(line)
                    self.move(line)
                    #self.driveMotors(0.1, 0.1)
            

            """if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()"""
            

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            print('Go')
            self.driveMotors(0.1,0.1)

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()