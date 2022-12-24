import struct
import math
import timeit
import random

def usr(robot):
	if robot.assigned_id==0:
		robot.set_led(100,0,0)
		r=.1
	elif robot.assigned_id==1:
		robot.set_led(0,100,0)
		r=.25
	else:
		robot.set_led(0,0,100)
		r=.4

	#scaler mulitiplyers for the vectors
	taxis_scale = 1
	rand_scale = .75
	repulsion_scale= 1
	
	while 1:

		pose = get_valid_pose(robot)#gets a valid pose that is not none 
		
		taxis = get_taxis_vec(pose) # generates a taxis unit vector for a source at (0,0)
		rand_vec= get_rand_unit_vec() # generates a random unit vector
		repulsion_vec = get_repulsion_vec(robot,r) # generates a repulsion vector which i chose to cap at 10 

		rand_and_taxis = add_vecs(multiply_vec(taxis_scale, taxis),multiply_vec(rand_scale, rand_vec)) #combining random and taxis

		tot = add_vecs(rand_and_taxis, multiply_vec(repulsion_scale, repulsion_vec)) #adding in repulsion

		turn(tot,robot) #turns to align the heading with the vector 
		
		#moves for 1 second
		robot.set_vel(75,75)
		move_time = 1
		timer = timeit.default_timer()
		while timeit.default_timer() - timer < move_time:
			i = 0
		robot.set_vel(0,0)
	


def get_rand_unit_vec():
	#used to generate a random unit vector 
	x = random.random() - .5
	y = random.random() -.5
	mag = math.sqrt(x ** 2 + y ** 2)
	if mag == 0: 
		return [0,0]
	direction = [x / mag, y / mag]
	result = [direction[0],direction[1]]
	return(result)

def get_taxis_vec(pose):
	# returns a taxis vector  for a source at (0,0) given a valid position for the robot 
	norm = math.sqrt(pose[0] ** 2 + pose[1] ** 2)
	if norm ==0 :
		return [0,0]
	direction = [-pose[0] / norm, -pose[1] / norm]
	result = [direction[0],direction[1]]
	return result

def get_valid_pose(robot):
	#this function is used t o make sure that the position recived by the program is not none which woudl occure due to the refresh rate of the robots
	pose = robot.get_pose()
	while pose is None:
		pose = robot.get_pose()
	return pose

def multiply_vec(num,vec):
	#multiplies a vector by a scalar
	new_vec = [num*vec[0],num*vec[1]]
	return new_vec

def add_vecs(vec1,vec2):
	#adds the components of the vector
	vx= vec1[0]+vec2[0]
	vy= vec1[1]+vec2[1]
	return [vx,vy]

def get_repulsion_vec(robot,r):
	#used to get the repulsion vector of a given robot
	rep_vec = [0,0]
	rep_scaler = 1000
	recived = []
	sensing_time = 1 
	timer = timeit.default_timer()
	while timeit.default_timer() - timer < sensing_time:
		pose = get_valid_pose(robot)
		robot.send_msg(struct.pack('ffi',pose[0],pose[1],robot.id))
		msgs = robot.recv_msg()
		if len(msgs) > 0:
				pose_rxed= struct.unpack('ffi', msgs[0][:12])
				pose = get_valid_pose(robot)
				dist = math.sqrt((pose_rxed[0]-pose[0])**2+(pose_rxed[1]-pose[1])**2)

				if dist < r: 
					weight = rep_scaler*(r-dist)
					rep_vec_t = [weight*(pose[0]-pose_rxed[0]),weight*(pose[1]-pose_rxed[1])]

					if pose_rxed[2] not in recived: #if the robot has not hear from a given robot, add it to the repulsion vector , used to help account for bais due to one way turns 
						recived.append(pose_rxed[2])
						rep_vec = add_vecs(rep_vec_t,rep_vec)
	return cap_rep_vec(rep_vec)
	# return rep_vec

def cap_rep_vec(vec):
	# this is the capping function for the repulsion vector described in the paper, I picked 10 
	cap = 10
	mag = math.sqrt((vec[0])**2+(vec[1])**2)
	while mag > cap:
				normalizer = cap/mag
				vec = multiply_vec(normalizer,vec)
				mag = math.sqrt((vec[0])**2+(vec[1])**2)
	return(vec)

def turn(vec,robot):
	#turns the robot until it's measured position is the same as what is desired 
	error = math.pi/24
	ang = math.atan2(vec[1],vec[0])
	t = get_valid_pose(robot)[2]
	while abs(t-ang)> error:
		robot.set_vel(70,-70)
		t = get_valid_pose(robot)[2]
	robot.set_vel(0,0)



