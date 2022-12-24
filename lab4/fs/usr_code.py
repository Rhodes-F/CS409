import struct
import math
import timeit
def usr(robot):
	while 1:
		vec = get_vectors(robot) #get the vector that is the sum of the migration vector,the repulsion vector, the allignment vector, and the cohesion vector
		ang = math.atan2(vec[1],vec[0]) #get the angle of the vector
		pose= get_valid_pose(robot)
		turn_to_ang(pose[2],ang,robot) #turn to the angle of the vector using the turn_to_ang function which should corrctly pick left or right turns


def get_valid_pose(robot):
	#this function is used t o make sure that the position recived by the program is not none which woudl occure due to the refresh rate of the robots
	pose = robot.get_pose()
	while pose is None:
		pose = robot.get_pose()
	return pose


def add_vecs(vec1,vec2):
	#adds the components of the vector
	vx= vec1[0]+vec2[0]
	vy= vec1[1]+vec2[1]
	return [vx,vy]

def multiply_vec(num,vec):
	#multiplies a vector by a scalar
	new_vec = [num*vec[0],num*vec[1]]
	return new_vec



def normalize_vec(vec):
	#normalizes a vector
	mag = math.sqrt(vec[0]**2+vec[1]**2)
	if mag == 0:
		return vec
	return [vec[0]/mag,vec[1]/mag]


def get_vectors(robot): 
	# this function does the bulk of the work fo this assignment 
	# it returns a vector that is the sum of the migration vector,the repulsion vector, the allignment vector, and the cohesion vector
	r= 5
	neighbors = []
	i = 0 #this will be a counter of messages recived
	rep_vec = [0,0]
	center_vec = [0,0]
	allignment_vec = [0,0]
	while i < 20: #while we have not recived 20 messages
		pos = get_valid_pose(robot)
		vec = [-pos[0],-pos[1]]
		mig_vec = normalize_vec(vec) #this is the migration vector
		robot.send_msg(struct.pack("fffi",pos[0],pos[1],pos[2],robot.id)) #send a message with the robots position and id
		msgs = robot.recv_msg()
		if len(msgs) > 0: #if we have recived a message
			x,y,theta,rId = struct.unpack('fffi',msgs[0][:16])
			pos = get_valid_pose(robot)
			i += 1 #increment the counter for messages recived
			dist = math.sqrt((x-pos[0])**2+(y-pos[1])**2)
			if dist < r: #if the robot is within the virtual communication radius
				weight = 1/(dist)
				rep_vect = [weight*(pos[0]-x),weight*(pos[1]-y)]
				if rId not in neighbors:
					neighbors.append(rId)
					allt = [math.cos(theta),math.sin(theta)]
					allignment_vec = add_vecs(allignment_vec,allt)
					center_vec = add_vecs(center_vec,[x,y])
					rep_vec = add_vecs(rep_vec,rep_vect)
		if i > 18: #if we have recived 19 messages whcih should be the max because there are 20 robots
			center_vec = multiply_vec((1/(len(neighbors)+1)),center_vec)
			alli_vec = [(allignment_vec[0] + math.cos(pos[2]))/(len(neighbors) + 1), \
						(allignment_vec[1] + math.sin(pos[2]))/(len(neighbors) + 1)]
			cohes_vec = [-(pos[0] - center_vec[0]),-(pos[1] - center_vec[1])]
			rep_vec = normalize_vec(rep_vec)
			cohes_vec = normalize_vec(cohes_vec)
			tot_vec = [cohes_vec[0] + .885*mig_vec[0] + 1.6*rep_vec[0] + alli_vec[0], \
						cohes_vec[1] + .885*mig_vec[1] + 1.6*rep_vec[1] + alli_vec[1]]
			return tot_vec #return the vector that is the sum of the migration vector,the repulsion vector, the allignment vector, and the cohesion vector

def turn_to_ang(t,ang,robot):
	
        e = t - ang
		#if the difference is negative and less than pi or if the difference is positive and greater than pi then we want to tuen left
        if (e < 0 and abs(e) < math.pi) or (e > 0 and abs(e) > math.pi): 
            turn_r = abs(e)
            turn_l = -turn_r
        else:
            turn_r = -abs(e)
            turn_l = -turn_r
		#the robot should always be moving forward but the turn rate will be determined by the difference between the current heading and the desired heading
        if abs(e) > 0.1:
            robot.set_vel(60 + turn_l*40,60 + turn_r*40)
            return 0
        else:
            robot.set_vel(100,100)

            return 1
