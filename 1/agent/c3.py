
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from time import sleep
import collections

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
	prevarg = ""
	init = True
	target_x, target_y = 0, 0
	c2_main_state = "info"
	c2_child_state = "seen"
	c2_prev_child_state = None
	mapa = [[' ' for i in range(28)] for j in range(56)] # guarda tipo de celula mapa[28][14] == 'X'
	seen = [[False for i in range(28)] for j in range(56)] # guarda celula vista? seen[28][14] == True
	curr_x, curr_y = 28, 14
	direction = "D" # Direita
	offset_x, offset_y = 0, 0
	circuitos = [ [(28,14)] ] # [C0]
	c2_mapping = "re" # encostar a esquerda
	queue = []
	rob_name = ""
	beacons = []
	G = []
	allpaths = []

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
			print(".", end='')
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

	def isWallInFront(self, measure):
		if measure > 1:
			return True
		else:
			return False
	
	def wallInFront(self, direction):
		if direction == 'D':
			return self.curr_x + 1, self.curr_y
		elif direction == 'C':
			return self.curr_x, self.curr_y + 1
		elif direction == 'E':
			return self.curr_x - 1, self.curr_y
		else: # direction == 'B'
			return self.curr_x, self.curr_y - 1
	
	def cellInFront(self, direction):
		if direction == 'D':
			return self.curr_x + 2, self.curr_y
		elif direction == 'C':
			return self.curr_x, self.curr_y + 2
		elif direction == 'E':
			return self.curr_x - 2, self.curr_y
		else: # direction == 'B'
			return self.curr_x, self.curr_y - 2

	# DETERMINES NEXT ORIENTATION
	# IF IM FACING 'Direita' AND WANT TO ROTATE RIGHT
	# I SHOULD FACE 'Baixo'
	def compassTurn(self, side):
		coords = ['D', 'C', 'E', 'B']
		idx = coords.index(self.direction)
		if side == 'right':
			#print(self.direction, 'right', coords[idx-1])
			return coords[idx-1]
		else:
			if idx+1 == len(coords):
				#print(self.direction, 'left', coords[0])
				return coords[0]
			else:
				#print(self.direction, 'left', coords[idx+1])
				return coords[idx+1]
	
	def distToTarget(self):
		if self.direction in ['D', 'E']:
			return abs( (self.measures.x - self.offset_x) - self.target_x )
		else: # self.direction in ['C', 'B']:
			return abs( (self.measures.y - self.offset_y) - self.target_y )

	def writeToFile(self):

		#self.mapa[28][14] = 'I'

		#f = open("a.out", "w")

		#for y in reversed(range(28)):
		#	if y != 0:
		#		for x in range(56):
		#			if x != 0:
		#				f.write( ''.join( self.mapa[x][y] ) )
		#		
		#		f.write("\n")


		#f.close()

		# CONSTRUIR GRAFO
		self.G = []
		for x in range(56):
			for y in range(28):
				if self.mapa[x][y] == 'X' and x%2==0 and y%2==0:
					st_1 = '' + str(x) + ',' + str(y)
					#G[st_1] = {}
					# D
					if self.mapa[x+1][y] == 'X' and self.mapa[x+2][y] == 'X':
						st_2 = '' + str((x+2)) + ',' + str(y)
						#G[st_1][st_2] = 10
						self.G.append((st_1, st_2, 1))
					# E
					if self.mapa[x-1][y] == 'X' and self.mapa[x-2][y] == 'X':
						st_2 = '' + str((x-2)) + ',' + str(y)
						#G[st_1][st_2] = 10
						self.G.append((st_1, st_2, 1))
					# C
					if self.mapa[x][y+1] == 'X' and self.mapa[x][y+2] == 'X':
						st_2 = '' + str(x) + ',' + str((y+2))
						#G[st_1][st_2] = 10
						self.G.append((st_1, st_2, 1))
					# B
					if self.mapa[x][y-1] == 'X' and self.mapa[x][y-2] == 'X':
						st_2 = '' + str(x) + ',' + str((y-2))
						#G[st_1][st_2] = 10
						self.G.append((st_1, st_2, 1))
		
		startlist = []
		for i in range(len(self.beacons)):
			if i != 0:
				startlist.append(i)
		
		lot = self.allcircuits([0, 0], 0, startlist)
		#print(lot)
		print()
		lowestcost = 100000
		bestpath = None
		for path in self.allpaths:
			print(path)
			if path[0] < lowestcost:
				bestpath = path
		
		# [custo, start, beacon1, ... , beaconx, start]

		circuit = []
		print()

		for i in range(len(bestpath)-1):
			if i != 0:
				print(i, self.beacons, bestpath)
				x1, y1 = self.beacons[int(bestpath[i])][0], self.beacons[int(bestpath[i])][1]
				x2, y2 = self.beacons[int(bestpath[i+1])][0], self.beacons[int(bestpath[i+1])][1]
				graph = Graph(self.G)
				s = '' + str(x1) + ',' + str((y1))
				v = '' + str(x2) + ',' + str((y2))
				path = list(collections.deque(graph.dijkstra(s, v)))
				if i != 1:
					path.pop(0)
				circuit += path
		
		f = open("b.out", "w")

		for cell in circuit:
			x = int(cell.split(',')[0])
			y = int(cell.split(',')[1])

			f.write( str(28-x) )
			f.write( ' ' )
			f.write( str(14-y) )
			if (x,y) in self.beacons:
				idx = self.beacons.index((x,y))
				if int(idx) != 0:
					f.write( ' #' + str(idx) )
			f.write( '\n' )

		f.close()


	def allcircuits(self, currpath, start, beaconlist, end=False):

		print("\n", start, "beaconlist", beaconlist)
		print(currpath)

		if end:
			self.allpaths.append(currpath)
			return []

		childpaths = []

		if len(beaconlist) == 0:
			beaconlist.append(0)
			end=True
			

		for beacon in beaconlist:
			sublist = []
			for sub in beaconlist:
				if sub != beacon:
					sublist.append(sub)
			x1, y1 = self.beacons[start][0], self.beacons[start][1]
			x2, y2 = self.beacons[beacon][0], self.beacons[beacon][1]
			graph = Graph(self.G)
			s = '' + str(x1) + ',' + str((y1))
			v = '' + str(x2) + ',' + str((y2))
			path = list(collections.deque(graph.dijkstra(s, v)))

			newpath = currpath.copy()
			newpath[0] += len(path)
			newpath.append(beacon)

			childpaths.append( (len(path), currpath, self.allcircuits(newpath, beacon, sublist, end)) )
		
		return (start, childpaths)



	def wander(self):

		if self.measures.time == 5000:
			self.finish()
			self.writeToFile()

		center = self.measures.irSensor[0]
		left = self.measures.irSensor[1]
		right = self.measures.irSensor[2]
		back = self.measures.irSensor[3]

		#print("")
		(closestWall_val, closestWall_nam) = self.closestWall(self.measures.irSensor)
		#print("Closest wall is", closestWall_nam, "->", closestWall_val)

		print("\n["+self.c2_main_state+"-"+self.c2_child_state+"-"+self.c2_mapping+"]", self.measures.compass, self.measures.x, self.measures.y)
		#print(self.measures.compass, self.measures.x, self.measures.x % 2)



		if self.init:
			self.cprint("START")
			#start_x, start_y = self.measures.x, self.measures.y
			
			#self.target_x, self.target_y = self.measures.x + 2, self.measures.y
			self.init = False
			self.mapa[self.curr_x][self.curr_y] = 'O'

			self.offset_x = self.measures.x - self.curr_x
			self.offset_y = self.measures.y - self.curr_y

			print(self.nBeacons, "beacons (including start)")
			for i in range(int(self.nBeacons)):
				self.beacons.append(None)
			
			self.beacons[0] = (28, 14)

			print("beacons", self.beacons)
				

		# self.direction (D, C, E, B) MAP ORIENTED
		# DIREITA -> 0
		# CIMA -> 90
		# ESQUERDA -> -180/180
		# BAIXO -> -90

		# info phase
		#     seen? -> (yes) -> go phase
		#           -> (no)  -> avaliar
		# go phase
		#     try left -> try front -> try right -> 180 -> go phase

		### STATE LOGIC ###
		if self.c2_main_state == 'info':
			if self.c2_child_state == 'seen':

				### CHECK IF SEEN ###
				if not( self.seen[self.curr_x][self.curr_y] ):

					### IF BEACON ###
					if self.measures.ground != -1:
						self.beacons[self.measures.ground] = (self.curr_x, self.curr_y)
						print("new beacon", self.measures.ground)
						print(self.beacons)

					### CHECK WALLS ###
					front_wall_x, front_wall_y = self.wallInFront(self.direction)
					left_direction = self.compassTurn('left')
					left_wall_x, left_wall_y = self.wallInFront(left_direction)
					right_direction = self.compassTurn('right')
					right_wall_x, right_wall_y = self.wallInFront(right_direction)

					### CHECK CELLS ###
					front_cell_x, front_cell_y = self.cellInFront(self.direction)
					left_direction = self.compassTurn('left')
					left_cell_x, left_cell_y = self.cellInFront(left_direction)
					right_direction = self.compassTurn('right')
					right_cell_x, right_cell_y = self.cellInFront(right_direction)

					# CHECK FRONT #
					if self.isWallInFront(center):
						if self.direction in ['D', 'E']:
							self.mapa[front_wall_x][front_wall_y] = '|'
							#print("Found |")
						else:
							self.mapa[front_wall_x][front_wall_y] = '-'
							#print("Found -")
					else:
						self.mapa[front_wall_x][front_wall_y] = 'X'
						self.mapa[front_cell_x][front_cell_y] = 'X'
					
					# CHECK LEFT #
					if self.isWallInFront(left):
						if left_direction in ['D', 'E']:
							self.mapa[left_wall_x][left_wall_y] = '|'
							#print("Found |")
						else:
							self.mapa[left_wall_x][left_wall_y] = '-'
							#print("Found -")
					else:
						self.mapa[left_wall_x][left_wall_y] = 'X'
						self.mapa[left_cell_x][left_cell_y] = 'X'

					# CHECK RIGHT #
					if self.isWallInFront(right):
						if right_direction in ['D', 'E']:
							self.mapa[right_wall_x][right_wall_y] = '|'
							#print("Found |")
						else:
							self.mapa[right_wall_x][right_wall_y] = '-'
							#print("Found -")
					else:
						self.mapa[right_wall_x][right_wall_y] = 'X'
						self.mapa[right_cell_x][right_cell_y] = 'X'
					
					self.seen[self.curr_x][self.curr_y] = True

				
				#for y in reversed(range(28)):
				#	for x in range(56):
				#		print( ''.join( self.mapa[x][y] ), end=''  )
				#	print("\n", end='')

				pos = (self.curr_x, self.curr_y)
				
				if len(self.circuitos) == 1: # AINDA SO HA C0
					if pos not in self.circuitos[0]: # NAO ESTAMOS NA ORIGEM
						self.circuitos.append([pos])
						#self.circuitos[1].append(pos)

				else:
					circuit_complete = False
					print(pos)
					#print(self.circuitos)
					if len(self.circuitos[-1]) != 0:
						for i in range(len(self.circuitos)-1):
							circuito = self.circuitos[i]
							print(circuito)
							if pos in circuito:
								circuit_complete = True
								break
					if circuit_complete:
						self.c2_mapping = 'search'
						print("[SEARCH]")
					else:
						self.circuitos[-1].append(pos)

					print(self.circuitos[-1])
				
					print("circuit_complete", len(self.circuitos)-1, circuit_complete)

				if self.c2_mapping == 're':
					self.c2_main_state = 'go'
					self.c2_child_state = 'left'
					#self.setNextState()


				if self.c2_mapping == 'search':
					# FAZER LISTA DE CANDIDATOS
					candidatos = []
					for x in range(56):
						for y in range(28):
							if not self.seen[x][y] and self.mapa[x][y] == 'X' and x%2==0 and y%2==0:
								candidatos.append( (x,y) )
					# DIJKSTRA EM CADA CANDIDATO

					G = []
					for x in range(56):
						for y in range(28):
							if self.mapa[x][y] == 'X' and x%2==0 and y%2==0:
								st_1 = '' + str(x) + ',' + str(y)
								#G[st_1] = {}
								# D
								if self.mapa[x+1][y] == 'X' and self.mapa[x+2][y] == 'X':
									st_2 = '' + str((x+2)) + ',' + str(y)
									#G[st_1][st_2] = 10
									G.append((st_1, st_2, 1))
								# E
								if self.mapa[x-1][y] == 'X' and self.mapa[x-2][y] == 'X':
									st_2 = '' + str((x-2)) + ',' + str(y)
									#G[st_1][st_2] = 10
									G.append((st_1, st_2, 1))
								# C
								if self.mapa[x][y+1] == 'X' and self.mapa[x][y+2] == 'X':
									st_2 = '' + str(x) + ',' + str((y+2))
									#G[st_1][st_2] = 10
									G.append((st_1, st_2, 1))
								# B
								if self.mapa[x][y-1] == 'X' and self.mapa[x][y-2] == 'X':
									st_2 = '' + str(x) + ',' + str((y-2))
									#G[st_1][st_2] = 10
									G.append((st_1, st_2, 1))
								

					print("CANDIDATOS", candidatos)
					#print(G)

					graph = Graph(G)

					best_target = None
					best_cost = 1000
					s = '' + str(pos[0]) + ',' + str(pos[1])
					for candidato in candidatos:
						v = '' + str(candidato[0]) + ',' + str(candidato[1])
						#path = shortestPath(G, s, v)
						
						path = list(collections.deque(graph.dijkstra(s, v)))
						print(path)

						#print("PATH", path)
						cost = len(path)
						# cost = tamanho de pos até candidato
						if cost < best_cost:
							best_cost = cost
							best_target = candidato
							self.queue = path
							self.queue.pop(0)


					if best_target == None:
						self.finish()
						self.writeToFile()
					else:

						# CRIAR NOVO CIRCUITO
						self.circuitos.append([])
						print("new circuit", len(self.circuitos)-1)
						self.c2_mapping = 'follow'


				if self.c2_mapping == 'follow':
					print("QUEUE", self.queue)
					if len(self.queue) == 0:
						self.c2_mapping = 're'
					else:
						# IR PARA TARGET

						tgt = self.queue.pop(0)
						print(str(self.curr_x)+","+str(self.curr_y)+" -> "+str(tgt))

						tx = int(tgt.split(',')[0])
						ty = int(tgt.split(',')[1])

						if tx > self.curr_x:
							target_direction = 'D'
						elif tx < self.curr_x:
							target_direction = 'E'
						elif ty > self.curr_y:
							target_direction = 'C'
						else:
							target_direction = 'B'
						
						print(self.direction, "->", target_direction)

						self.c2_main_state = 'go'
						if self.direction == target_direction:
							self.c2_child_state = 'front'
						else:
							if self.compassTurn('left') == target_direction:
								self.c2_child_state = 'left'
							elif self.compassTurn('right') == target_direction:
								self.c2_child_state = 'right'
							else:
								self.c2_child_state = '180'


		elif self.c2_main_state == 'go':

			# go phase
			#     try left -> try front -> try right -> 180 -> go phase

			if self.c2_child_state == 'rr':

				target_direction = self.compassTurn('right')
				self.target_x, self.target_y = self.cellInFront(target_direction)

				dt_x = abs(self.measures.x - (self.target_x + self.offset_x) )
				dt_y = abs(self.measures.y - (self.target_y + self.offset_y) )
				#print (dt_y, 'y /', dt_x, 'x')
				if dt_x == 0:
					dt_x = 0.00001
				if dt_y == 0:
					dt_y = 0.00001
				
				alpha = (  atan(dt_y / dt_x) * 180  ) / pi
				#print('a =', alpha, 'º')

				#alpha_min = alpha - 5
				#alpha_max = alpha + 5

				if target_direction == 'D':
					
					if self.target_y + self.offset_y > self.measures.y:
						theta = alpha
					else:
						theta = -alpha
					
					#if abs(self.measures.compass) < 12.5:
					#	self.driveMotors(0,0)
					#	#self.setNextState()
					#	self.direction = target_direction
					#	self.c2_child_state = 'drive'
					#elif abs(self.measures.compass) < 30:
					#	self.driveMotors(-0.05, 0.05)
					#else:
					#	self.driveMotors(-0.1, 0.1)

				elif target_direction == 'C':

					if self.target_x + self.offset_x < self.measures.x:
						theta = 180 - alpha
					else:
						theta = alpha
					
					#if 95 > self.measures.compass > 77.5:
					#	self.driveMotors(0,0)
					#	#self.setNextState()
					#	self.direction = target_direction
					#	self.c2_child_state = 'drive'
					#elif 120 > self.measures.compass > 60:
					#	self.driveMotors(-0.05, 0.05)
					#else:
					#	self.driveMotors(-0.1, 0.1)

				elif target_direction == 'E':

					if self.target_y + self.offset_y > self.measures.y:
						theta = 180 - alpha
					else:
						theta = -180 + alpha
					
					if theta < 0: # -179...
						if theta - 30 < -180:
							tmp = 180 - abs(theta)
							theta_max_near = 180 - (30-tmp)
							if theta - 5 < -180:
								theta_max = 180 - (5-tmp)
								#print(theta_max, '= 180 - ( 5 -', tmp, ')')
							else:
								theta_max = theta - 5
						else:
							theta_max = theta - 5
							theta_max_near = theta - 30

						theta_min = theta + 5
						theta_min_near = theta + 30

					else: # +179...
						if theta + 30 > 180:
							tmp = 180 - theta
							theta_min_near = -180 + (30-tmp)
							if theta + 5 > 180:
								theta_min = -180 + (5-tmp)
							else:
								theta_min = theta + 5
						else:
							theta_min = theta + 5
							theta_min_near = theta + 30
						
						theta_max = theta - 5
						theta_max_near = theta - 30

					#print(theta_max, '> theta >', theta_min)
					#print(theta_max_near, '> theta >', theta_min_near)

					#if 180 - abs(self.measures.compass) < 12.5:
					#	self.driveMotors(0,0)
					#	#self.setNextState()
					#	self.direction = target_direction
					#	self.c2_child_state = 'drive'
					#elif 180 - abs(self.measures.compass) < 30:
					#	self.driveMotors(-0.05, 0.05)
					#else:
					#	self.driveMotors(-0.1, 0.1)

				else: #if target_direction == 'B':

					if self.target_x + self.offset_x < self.measures.x:
						theta = -180 + alpha
					else:
						theta = -alpha

					#if -95 < self.measures.compass < -77.5:
					#	self.driveMotors(0,0)
					#	#self.setNextState()
					#	self.direction = target_direction
					#	self.c2_child_state = 'drive'
					#elif -120 < self.measures.compass < -60:
					#	self.driveMotors(-0.05, 0.05)
					#else:
					#	self.driveMotors(-0.1, 0.1)

				#print('t =', theta, 'º')

				if target_direction != 'E':
					theta_min = theta - 5
					theta_max = theta + 5

					theta_min_near = theta - 30
					theta_max_near = theta + 30

					if theta_max > self.measures.compass > theta_min:
						#print("STOP")
						self.driveMotors(0,0)
						self.direction = target_direction
						self.c2_child_state = 'drive'
					elif theta_max_near > self.measures.compass > theta_min_near:
						#print("SLOW")
						self.driveMotors(0.015, -0.015)
					else:
						#print(" GO ")
						self.driveMotors(0.15, -0.15)

				else:

					flag = False

					# AMBOS O MESMO SINAL
					if (theta_max < 0 and theta_min < 0):
						if theta_max < self.measures.compass < theta_min:
							#print("STOP")
							self.driveMotors(0,0)
							self.direction = target_direction
							self.c2_child_state = 'drive'
							flag = True
					else:
						if self.measures.compass > theta_max or self.measures.compass < theta_min:
							#print("STOP")
							self.driveMotors(0,0)
							self.direction = target_direction
							self.c2_child_state = 'drive'
							flag = True
					
					if (theta_max_near < 0 and theta_min_near < 0) and not flag:
						if theta_max_near < self.measures.compass < theta_min_near:
							#print("SLOW")
							self.driveMotors(0.015, -0.015)
							flag = True
					elif not flag:
						if self.measures.compass > theta_max_near or self.measures.compass < theta_min_near:
							#print("SLOW")
							self.driveMotors(0.015, -0.015)
							flag = True
					
					if not flag:
						#print(" GO ")
						self.driveMotors(0.15, -0.15)

				#target_direction = self.compassTurn('right')
				#if target_direction == 'D':
				#	if abs(self.measures.compass) < 5:
				#		self.driveMotors(0,0)
				#		#self.setNextState()
				#		self.direction = target_direction
				#		self.c2_child_state = 'drive'
				#	elif abs(self.measures.compass) < 30:
				#		self.driveMotors(0.05, -0.05)
				#	else:
				#		self.driveMotors(0.1, -0.1)

				#elif target_direction == 'C':
				#	if 95 > self.measures.compass > 85:
				#		self.driveMotors(0,0)
				#		#self.setNextState()
				#		self.direction = target_direction
				#		self.c2_child_state = 'drive'
				#	elif 120 > self.measures.compass > 60:
				#		self.driveMotors(0.05, -0.05)
				#	else:
				#		self.driveMotors(0.1, -0.1)

				#elif target_direction == 'E':
				#	if 180 - abs(self.measures.compass) < 5:
				#		self.driveMotors(0,0)
				#		#self.setNextState()
				#		self.direction = target_direction
				#		self.c2_child_state = 'drive'
				#	elif 180 - abs(self.measures.compass) < 30:
				#		self.driveMotors(0.05, -0.05)
				#	else:
				#		self.driveMotors(0.1, -0.1)

				#elif target_direction == 'B':
				#	if -95 < self.measures.compass < -85:
				#		self.driveMotors(0,0)
				#		#self.setNextState()
				#		self.direction = target_direction
				#		self.c2_child_state = 'drive'
				#	elif -120 < self.measures.compass < -60:
				#		self.driveMotors(0.05, -0.05)
				#	else:
				#		self.driveMotors(0.1, -0.1)
			
			elif self.c2_child_state == 're':

				target_direction = self.compassTurn('left')
				self.target_x, self.target_y = self.cellInFront(target_direction)

				dt_x = abs(self.measures.x - (self.target_x + self.offset_x) )
				dt_y = abs(self.measures.y - (self.target_y + self.offset_y) )
				#print (dt_y, 'y /', dt_x, 'x')
				if dt_x == 0:
					dt_x = 0.00001
				if dt_y == 0:
					dt_y = 0.00001
				
				alpha = (  atan(dt_y / dt_x) * 180  ) / pi
				#print('a =', alpha, 'º')

				#alpha_min = alpha - 5
				#alpha_max = alpha + 5

				if target_direction == 'D':
					
					if self.target_y + self.offset_y > self.measures.y:
						theta = alpha
					else:
						theta = -alpha
					
					#if abs(self.measures.compass) < 12.5:
					#	self.driveMotors(0,0)
					#	#self.setNextState()
					#	self.direction = target_direction
					#	self.c2_child_state = 'drive'
					#elif abs(self.measures.compass) < 30:
					#	self.driveMotors(-0.05, 0.05)
					#else:
					#	self.driveMotors(-0.1, 0.1)

				elif target_direction == 'C':

					if self.target_x + self.offset_x < self.measures.x:
						theta = 180 - alpha
					else:
						theta = alpha
					
					#if 95 > self.measures.compass > 77.5:
					#	self.driveMotors(0,0)
					#	#self.setNextState()
					#	self.direction = target_direction
					#	self.c2_child_state = 'drive'
					#elif 120 > self.measures.compass > 60:
					#	self.driveMotors(-0.05, 0.05)
					#else:
					#	self.driveMotors(-0.1, 0.1)

				elif target_direction == 'E':

					if self.target_y + self.offset_y > self.measures.y:
						theta = 180 - alpha
					else:
						theta = -180 + alpha
					
					if theta < 0: # -179...
						if theta - 30 < -180:
							tmp = 180 - abs(theta)
							theta_max_near = 180 - (30-tmp)
							if theta - 5 < -180:
								theta_max = 180 - (5-tmp)
								#print(theta_max, '= 180 - ( 5 -', tmp, ')')
							else:
								theta_max = theta - 5
						else:
							theta_max = theta - 5
							theta_max_near = theta - 30

						theta_min = theta + 5
						theta_min_near = theta + 30

					else: # +179...
						if theta + 30 > 180:
							tmp = 180 - theta
							theta_min_near = -180 + (30-tmp)
							if theta + 5 > 180:
								theta_min = -180 + (5-tmp)
							else:
								theta_min = theta + 5
						else:
							theta_min = theta + 5
							theta_min_near = theta + 30
						
						theta_max = theta - 5
						theta_max_near = theta - 30

					#print(theta_max, '> theta >', theta_min)
					#print(theta_max_near, '> theta >', theta_min_near)

					#if 180 - abs(self.measures.compass) < 12.5:
					#	self.driveMotors(0,0)
					#	#self.setNextState()
					#	self.direction = target_direction
					#	self.c2_child_state = 'drive'
					#elif 180 - abs(self.measures.compass) < 30:
					#	self.driveMotors(-0.05, 0.05)
					#else:
					#	self.driveMotors(-0.1, 0.1)

				else: #if target_direction == 'B':

					if self.target_x + self.offset_x < self.measures.x:
						theta = -180 + alpha
					else:
						theta = -alpha

					#if -95 < self.measures.compass < -77.5:
					#	self.driveMotors(0,0)
					#	#self.setNextState()
					#	self.direction = target_direction
					#	self.c2_child_state = 'drive'
					#elif -120 < self.measures.compass < -60:
					#	self.driveMotors(-0.05, 0.05)
					#else:
					#	self.driveMotors(-0.1, 0.1)

				#print('t =', theta, 'º')

				if target_direction != 'E':
					theta_min = theta - 5
					theta_max = theta + 5

					theta_min_near = theta - 30
					theta_max_near = theta + 30

					if theta_max > self.measures.compass > theta_min:
						#print("STOP")
						self.driveMotors(0,0)
						self.direction = target_direction
						self.c2_child_state = 'drive'
					elif theta_max_near > self.measures.compass > theta_min_near:
						#print("SLOW")
						self.driveMotors(-0.015, 0.015)
					else:
						#print(" GO ")
						self.driveMotors(-0.15, 0.15)

				else:

					flag = False

					# AMBOS O MESMO SINAL
					if (theta_max < 0 and theta_min < 0):
						if theta_max < self.measures.compass < theta_min:
							#print("STOP")
							self.driveMotors(0,0)
							self.direction = target_direction
							self.c2_child_state = 'drive'
							flag = True
					else:
						if self.measures.compass > theta_max or self.measures.compass < theta_min:
							#print("STOP")
							self.driveMotors(0,0)
							self.direction = target_direction
							self.c2_child_state = 'drive'
							flag = True
					
					if (theta_max_near < 0 and theta_min_near < 0) and not flag:
						if theta_max_near < self.measures.compass < theta_min_near:
							#print("SLOW")
							self.driveMotors(-0.015, 0.015)
							flag = True
					elif not flag:
						if self.measures.compass > theta_max_near or self.measures.compass < theta_min_near:
							#print("SLOW")
							self.driveMotors(-0.015, 0.015)
							flag = True
					
					if not flag:
						#print(" GO ")
						self.driveMotors(-0.15, 0.15)

			

			elif self.c2_child_state == '180':
				target_direction = self.compassTurn('left')
				if target_direction == 'D':
					if abs(self.measures.compass) < 5:
						self.driveMotors(0,0)
						#self.setNextState()
						self.direction = target_direction
						self.c2_child_state = 're'
					elif abs(self.measures.compass) < 30:
						self.driveMotors(-0.015, 0.015)
					else:
						self.driveMotors(-0.15, 0.15)

				elif target_direction == 'C':
					if 95 > self.measures.compass > 85:
						self.driveMotors(0,0)
						#self.setNextState()
						self.direction = target_direction
						self.c2_child_state = 're'
					elif 120 > self.measures.compass > 60:
						self.driveMotors(-0.015, 0.015)
					else:
						self.driveMotors(-0.15, 0.15)

				elif target_direction == 'E':
					if 180 - abs(self.measures.compass) < 5:
						self.driveMotors(0,0)
						#self.setNextState()
						self.direction = target_direction
						self.c2_child_state = 're'
					elif 180 - abs(self.measures.compass) < 30:
						self.driveMotors(-0.015, 0.015)
					else:
						self.driveMotors(-0.15, 0.15)

				elif target_direction == 'B':
					if -95 < self.measures.compass < -85:
						self.driveMotors(0,0)
						#self.setNextState()
						self.direction = target_direction
						self.c2_child_state = 're'
					elif -120 < self.measures.compass < -60:
						self.driveMotors(-0.015, 0.015)
					else:
						self.driveMotors(-0.15, 0.15)

			elif self.c2_child_state == 'left':

				if not self.isWallInFront(left):
					self.c2_child_state = 're'
				else:
					self.c2_child_state = 'front'

			elif self.c2_child_state == 'front':

				if not self.isWallInFront(center):
					self.c2_child_state = 'drive'
				else:
					self.c2_child_state = 'right'

			elif self.c2_child_state == 'right':

				if not self.isWallInFront(right):
					self.c2_child_state = 'rr'
				else:
					self.c2_child_state = '180'

			elif self.c2_child_state == 'drive':
				#print(self.measures.x - self.target_x, self.measures.x, self.target_x)
				
				### GO/STOP MECHANICS ###

				self.target_x, self.target_y = self.cellInFront(self.direction)

				# TARGET HERE #
				if(self.distToTarget() < 0.05):
					#self.cprint('Sleep')
					self.driveMotors(0,0)
					#sleep(1)
					self.c2_main_state = 'info'
					self.c2_child_state = 'seen'
					self.curr_x = self.target_x
					self.curr_y = self.target_y

					
					
					#sleep(1)

				# TARGET CLOSE #
				elif (self.distToTarget() < 0.5):
					self.cprint('[Go] Slow')
					self.driveMotors(0.035,0.035)
					#self.driveMotors(0.025,0.025)

				# TARGET FAR #
				else:

					target_direction = self.direction
					print(self.direction)
					self.target_x, self.target_y = self.cellInFront(target_direction)

					dt_x = abs(self.measures.x - (self.target_x + self.offset_x) )
					dt_y = abs(self.measures.y - (self.target_y + self.offset_y) )
					#print (dt_y, 'y /', dt_x, 'x')
					if dt_x == 0:
						dt_x = 0.00001
					if dt_y == 0:
						dt_y = 0.00001
					
					alpha = (  atan(dt_y / dt_x) * 180  ) / pi
					#print('a =', alpha, 'º')

					if target_direction == 'D':
						
						if self.target_y + self.offset_y > self.measures.y:
							print("target up")
							theta = alpha
						else:
							print("target down")
							theta = -alpha

					elif target_direction == 'C':

						if self.target_x + self.offset_x < self.measures.x:
							theta = 180 - alpha
						else:
							theta = alpha

					elif target_direction == 'E':

						if self.target_y + self.offset_y > self.measures.y:
							theta = 180 - alpha
						else:
							theta = -180 + alpha
						
						if theta > 0:
							theta = 180 - theta
						else:
							theta = -180 - theta
						
						#if theta < 0: # -179...
						#	if theta - 30 < -180:
						#		tmp = 180 - abs(theta)
						#		theta_max_near = 180 - (30-tmp)
						#		if theta - 2 < -180:
						#			theta_max = 180 - (2-tmp)
						#			#print(theta_max, '= 180 - ( 2 -', tmp, ')')
						#		else:
						#			theta_max = theta - 2
								
		
						#	else:
						#		theta_max = theta - 2
						#		theta_max_near = theta - 30

						#	theta_min = theta - 2
						#	theta_min_near = theta - 30

						#else: # +179...
						#	if theta + 30 > 180:
						#		tmp = 180 - theta
						#		theta_min_near = -180 + (30-tmp)
								
						#		if theta + 2 > 180:
						#			theta_min = -180 + (2-tmp)
						#		else:
						#			theta_min = theta + 2

						#	else:
						#		theta_min = theta - 2
						#		theta_min_near = theta - 30
							
						#	theta_max = theta + 2
						#	theta_max_near = theta + 30

						#print(theta_max, '> theta >', theta_min)
						#print(theta_max_near, '> theta >', theta_min_near)

					else: #if target_direction == 'B':

						if self.target_x + self.offset_x < self.measures.x:
							theta = -180 + alpha
						else:
							theta = -alpha

					#print('t =', theta, 'º')

					flag = False
					go = True

					#if target_direction != 'E':
					if True:

						if target_direction == 'E':
							if self.measures.compass > 0:
								compass = 180 - self.measures.compass
							else:
								compass = -180 - self.measures.compass
						else:
							compass = self.measures.compass

						theta_min = theta - 2
						theta_max = theta + 2

						theta_min_near = theta - 30
						theta_max_near = theta + 30

						#print("ct =", compass)
						#print(theta_max, '> theta >', theta_min)
						#print(theta_max_near, '> theta >', theta_min_near)

						if theta_max > compass > theta_min:
							#print("STOP")
							#self.driveMotors(0,0)
							self.direction = target_direction
							self.c2_child_state = 'drive'
						elif theta_max_near > compass > theta_min_near:
							#print("SLOW")
							if compass < theta:
								if target_direction != 'E':
									self.driveMotors(-0.01, 0.01)
								else:
									self.driveMotors(0.01, -0.01)
								#print("e")
							else:
								if target_direction != 'E':
									self.driveMotors(0.01, -0.01)
								else:
									self.driveMotors(-0.01, 0.01)
								#print("d")
							flag = True
							go = False
						else:
							#print(" GO ")
							if compass < theta:
								if target_direction != 'E':
									self.driveMotors(-0.1, 0.1)
								else:
									self.driveMotors(0.1, -0.1)
								#print("e")
							else:
								if target_direction != 'E':
									self.driveMotors(0.1, -0.1)
								else:
									self.driveMotors(-0.1, 0.1)
								#print("d")
							flag = True
							go = False

					else:

						

						if False:

							# AMBOS O MESMO SINAL
							if (theta_max < 0 and theta_min < 0):
								if theta_max < self.measures.compass < theta_min:
									print(".STOP")
									#self.driveMotors(0,0)
									self.direction = target_direction
									self.c2_child_state = 'drive'
									flag = True
							elif (theta_max > 0 and theta_min > 0):
								if theta_max > self.measures.compass > theta_min:
									print(".STOP")
									#self.driveMotors(0,0)
									self.direction = target_direction
									self.c2_child_state = 'drive'
									flag = True
							
							else:
								if self.measures.compass < 0 and theta < 0:
									if self.measures.compass < theta_min:
										print("..STOP.")
										#self.driveMotors(0,0)
										self.direction = target_direction
										self.c2_child_state = 'drive'
										flag = True
								elif self.measures.compass > 0 and theta > 0:
									if self.measures.compass > theta_min:
										print("..STOP..")
										#self.driveMotors(0,0)
										self.direction = target_direction
										self.c2_child_state = 'drive'
										flag = True
								else:
									if abs(self.measures.compass) > abs(theta_min):
										print("...STOP..")
										#self.driveMotors(0,0)
										self.direction = target_direction
										self.c2_child_state = 'drive'
										flag = True
								#if self.measures.compass > theta_max or self.measures.compass < theta_min:
								#	print("..STOP")
								#	#self.driveMotors(0,0)
								#	self.direction = target_direction
								#	self.c2_child_state = 'drive'
								#	flag = True
							
							if (theta_max_near < 0 and theta_min_near < 0) and not flag:
								if theta_max_near < self.measures.compass < theta_min_near:
									print(".SLOW")
									if (self.measures.compass < 0 and theta < 0) or (self.measures.compass > 0 and theta > 0):
										if self.measures.compass < theta:
											self.driveMotors(-0.01, 0.01)
											print("e")
										else:
											self.driveMotors(0.01, -0.01)
											print("d")
									elif self.measures.compass > 0:
										self.driveMotors(-0.01, 0.01)
										print("e")
									else:
										self.driveMotors(0.01, -0.01)
										print("d")
									flag = True
									go = False
							if (theta_max_near > 0 and theta_min_near > 0) and not flag:
								if theta_max_near > self.measures.compass > theta_min_near:
									print(".SLOW")
									if (self.measures.compass < 0 and theta < 0) or (self.measures.compass > 0 and theta > 0):
										if self.measures.compass < theta:
											self.driveMotors(-0.01, 0.01)
											print("e")
										else:
											self.driveMotors(0.01, -0.01)
											print("d")
									elif self.measures.compass > 0:
										self.driveMotors(-0.01, 0.01)
										print("e")
									else:
										self.driveMotors(0.01, -0.01)
										print("d")
									flag = True
									go = False

							elif not flag:
								if self.measures.compass > theta_max_near or self.measures.compass < theta_min_near:
									print("..SLOW")
									if (self.measures.compass < 0 and theta < 0) or (self.measures.compass > 0 and theta > 0):
										if self.measures.compass < theta:
											self.driveMotors(-0.01, 0.01)
											print("e")
										else:
											self.driveMotors(0.01, -0.01)
											print("d")
									elif self.measures.compass > 0:
										self.driveMotors(-0.01, 0.01)
										print("e")
									else:
										self.driveMotors(0.01, -0.01)
										print("d")
									flag = True
									go = False
							
							if not flag:
								print(" GO ")
								if (self.measures.compass < 0 and theta < 0) or (self.measures.compass > 0 and theta > 0):
									if self.measures.compass < theta:
										self.driveMotors(-0.1, 0.1)
										print("e")
									else:
										self.driveMotors(0.1, -0.1)
										print("d")
								elif self.measures.compass > 0:
									self.driveMotors(-0.1, 0.1)
									print("e")
								else:
									self.driveMotors(0.1, -0.1)
									print("d")
								go = False

					if go:
						self.cprint('[Go] Speed')
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
	elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
		rob_name = sys.argv[i + 1]
	elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
		mapc = Map(sys.argv[i + 1])
	else:
		print("Unkown argument", sys.argv[i])
		quit()

#https://morioh.com/p/ec5873a4c9f5

from collections import deque, namedtuple


# we'll use infinity as a default distance to nodes.
inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')


def make_edge(start, end, cost=1):
  return Edge(start, end, cost)


class Graph:
    def __init__(self, edges):
        # let's check that the data is right
        wrong_edges = [i for i in edges if len(i) not in [2, 3]]
        if wrong_edges:
            raise ValueError('Wrong edges data: {}'.format(wrong_edges))

        self.edges = [make_edge(*edge) for edge in edges]

    @property
    def vertices(self):
        return set(
            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def get_node_pairs(self, n1, n2, both_ends=True):
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    def remove_edge(self, n1, n2, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(n1, n2))

        self.edges.append(Edge(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Edge(start=n2, end=n1, cost=cost))

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def dijkstra(self, source, dest):
        assert source in self.vertices, 'Such source node doesn\'t exist'
        distances = {vertex: inf for vertex in self.vertices}
        previous_vertices = {
            vertex: None for vertex in self.vertices
        }
        distances[source] = 0
        vertices = self.vertices.copy()

        while vertices:
            current_vertex = min(
                vertices, key=lambda vertex: distances[vertex])
            vertices.remove(current_vertex)
            if distances[current_vertex] == inf:
                break
            for neighbour, cost in self.neighbours[current_vertex]:
                alternative_route = distances[current_vertex] + cost
                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    previous_vertices[neighbour] = current_vertex

        path, current_vertex = deque(), dest
        while previous_vertices[current_vertex] is not None:
            path.appendleft(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        if path:
            path.appendleft(current_vertex)
        return path


host = "localhost"
pos = 0
mapc = None
rob_name = "pClient3"
filename = "wtf.out"

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == '-r' or sys.argv[i] == "--robname") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        filename = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()


if __name__ == '__main__':
	rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
	if mapc != None:
		rob.setMap(mapc.labMap)
		rob.printMap()
	
	rob.run()
