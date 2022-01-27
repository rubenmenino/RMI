# -*- coding: utf-8 -*-
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
	prevarg = ""

	def __init__(self, rob_name, rob_id, angles, host):
		CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
		self.rob_name = rob_name

	# In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
	# to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
	def setMap(self, labMap):
		self.labMap = labMap

	def printMap(self):
		for l in reversed(self.labMap):
			print(''.join([str(l) for l in l]))

	def run(self):
		if self.status != 0:
			print("Connection refused or error")
			quit()

		state = 'stop'
		stopped_state = 'run'

		while True:
			self.readSensors()

			if self.measures.endLed:
				print(self.rob_name + " exiting")
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
				self.wander()
	
	



	def closestWall(self, irSensor):
		center = self.measures.irSensor[0]
		left = self.measures.irSensor[1]
		right = self.measures.irSensor[2]
		back = self.measures.irSensor[3]

		closest_val = center
		closest_nam = "Center"

		if left > closest_val:
			closest_val = left
			closest_nam = "Left"
		if right > closest_val:
			closest_val = right
			closest_nam = "Right"
		if back > closest_val:
			closest_val = back
			closest_nam = "Back"
		
		return (closest_val, closest_nam)
		
	def cprint(self, arg):
		if arg == self.prevarg:
			print('.')#, end='')
		else:
			print("")
			print(arg)
			self.prevarg = arg

	def sensorsWithinError(self, s1, s2):
		if s1 == 0 or s2 == 0: # NECESSÁRIO PARA N DAR ERRO div0
			return False
		difference = (1/s1) - (1/s2)
		#print("%.2f" % float(1/s1), "%.2f" % float(1/s2), "%.2f" % float(difference))
		if difference > -0.075 and difference < 0.075:
			return True
		else:
			return False

	def wander(self):

		if self.measures.time == 5000:
			self.finish()

		center = self.measures.irSensor[0]
		left = self.measures.irSensor[1]
		right = self.measures.irSensor[2]
		back = self.measures.irSensor[3]

		print("AVERAGE CENTER {}\nAVERAGE LEFT {}\nAVERAGE RIGHT {}\nAVERAGE BACK {}".format(center, left, right, back))
		#print("")
		(closestWall_val, closestWall_nam) = self.closestWall(self.measures.irSensor)
		#print("Closest wall is", closestWall_nam, "->", closestWall_val)
		print("VER -> ", closestWall_val, closestWall_nam)
		if closestWall_val > 6.6: # Wall is very near (< 0.15d)

			if closestWall_nam == "Left":
			#	self.cprint('[Near] L Rotate right')
				self.driveMotors(+0.1,-0.1)
				#self.driveMotors(+0.05,-0.05)

			elif closestWall_nam == "Right":
			#	self.cprint('[Near] R Rotate left')
				self.driveMotors(-0.1,+0.1)
				#self.driveMotors(-0.05,+0.05)

			else: # closestWall_nam == "Center":

				if left > right:
			#		self.cprint('[Near] C Rotate right')
					self.driveMotors(+0.85,0.05)
					#self.driveMotors(+0.05,-0.05)
				else:
			#		self.cprint('[Near] C Rotate left')
					self.driveMotors(0.05,+0.85)
					#self.driveMotors(-0.05,+0.05)
		

		elif closestWall_val > 2.857: # Wall is mildly near (< 0.35d)

			if closestWall_nam == "Left":

				#x = 1/closestWall_val
				#factor = 0.625 * x * x - 0.2 * x + 0.0309375

			#	self.cprint('[M-Near] L Rotate right')#+str(4*factor)+str(-3*factor))
				#self.driveMotors(4*factor,-1*factor)
				#self.driveMotors(+0.15,-0.125)
				self.driveMotors(+0.08,-0.06)
				#self.driveMotors(+0.06,-0.02)

			elif closestWall_nam == "Right":

				#x = 1/closestWall_val
				#factor = 0.625 * x * x - 0.2 * x + 0.0309375

			#	self.cprint('[M-Near] R Rotate left')#+str(-3*factor)+str(4*factor))
				#self.driveMotors(-1*factor,4*factor)
				#self.driveMotors(-0.125,+0.15)
				self.driveMotors(-0.06,+0.08)
				#self.driveMotors(-0.02,+0.06)

			else: # closestWall_nam == "Center":

				if left > right:
			#		self.cprint('[M-Near] C Rotate right')#+str(3*factor)+str(-1*factor))
					#self.driveMotors(3*factor,-1*factor)
					self.driveMotors(+0.12,-0.04)
					#self.driveMotors(+0.06,-0.02)
				else:
			#		self.cprint('[M-Near] C Rotate left')#+str(-1*factor)+str(3*factor))
					#self.driveMotors(-1*factor,3*factor)
					self.driveMotors(-0.04,+0.012)
					#self.driveMotors(-0.02,+0.06)


		else: # Wall is not near

			withinError = self.sensorsWithinError(left, right)
			if not withinError:

				if left > right:
			#		self.cprint('[Free] L Go right')
					#self.driveMotors(+0.15,+0.06)
					self.driveMotors(+0.15,-0.04)
				else:
			#		self.cprint('[Free] R Go left')
					#self.driveMotors(+0.06,+0.15)
					self.driveMotors(-0.04,+0.15)

			else:

			#	self.cprint('[Go] Speed')
				self.driveMotors(0.15,0.15)




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
