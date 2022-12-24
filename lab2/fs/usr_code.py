def usr(robot):
	import struct
	import timeit
	hop_count_l = 1000
	hop_count_r = 1000
	timer = timeit.default_timer()
	while timeit.default_timer() - timer < 10:
		if robot.assigned_id == 1 :
			robot.send_msg(struct.pack('ii', 0,0))# send hop count
			# ('robot L pos', (-1.0, -1.0, 0.0))
		elif robot.assigned_id == 2 :
			robot.send_msg(struct.pack('ii',0, 1))# send hop count
			# ('robot R pos', (0.20000000298023224, -0.9449999928474426, 0.0))
		else:
			msgs = robot.recv_msg()
			if len(msgs) > 0:
				curr_hop= struct.unpack('ii', msgs[0][:8])
				if curr_hop[1] == 0: #if you get a message from the left
					new_hop_l= curr_hop[0]
					if new_hop_l<hop_count_l: #if the hop count you recived is the lowest you have heard make that your hop count and send out hop count +1
						hop_count_l= new_hop_l
					robot.send_msg(struct.pack('ii', hop_count_l+1,0))# send hop count
				
				if curr_hop[1] == 1:#if you get a message from the right, the steps above are just repeated 
					new_hop_r= curr_hop[0]
					if new_hop_r<hop_count_r:
						hop_count_r= new_hop_r
					robot.send_msg(struct.pack('ii', hop_count_r+1,1))# send hop count

	######## COMMENT OR UNCOMMENT THESE LINES TO GET THE N ########
	make_smooth_n(robot, hop_count_l, hop_count_r, timer)
	# make_unsmooth_n(robot, hop_count_l, hop_count_r)




def make_unsmooth_n(robot, hop_count_l, hop_count_r):
	pos = get_cords(hop_count_l,hop_count_r)
	if pos[0]< (-.99):
		robot.set_led(100,100,100)
	elif pos[0]>.19:
		robot.set_led(100,100,100)
	elif pos[1]>(-1.2*pos[0]-.8) and pos[1]<(-1.2*pos[0]-.2):
		robot.set_led(100,100,100)


def get_cords(hop_count_l,hop_count_r):
	# a function to assign a coordinate system to the robots 
	# the position of the seeds were found as('robot 1 pos', (-1.0, -1.0, 0.0)) and ('robot 2 pos', (0.20000000298023224, -0.9449999928474426, 0.0))
	import math
	pos = None
	prev_error = 100000 # start by making the error large so that the robots will use the first position 
	for x in range(-100, 100):
		for y in range(-100, 400):	#increment throught the values for x and y 
			x_in_range, y_in_range = x / 100.0, y / 100.0 # this a a rescaling step so that the x and y are in the desired ranges with the desired steps 
			dist_l = math.sqrt((x_in_range - (-1))**2 + (y_in_range - (-1))**2)	#finding the distange between the seeds and current x and y
			dist_r = math.sqrt((x_in_range - (0.20000000298023224))**2 + (y_in_range - (-0.9449999928474426))**2)
			error = abs(dist_l - (hop_count_l*.13)) + abs(dist_r - (hop_count_r*.13)) # checking distance calculated above vs the hop count* the signal range
			if error < prev_error: #new best position
				prev_error = error
				pos = (x_in_range, y_in_range) #make that current position
	return pos


def make_smooth_n(robot, hop_count_l, hop_count_r, timer):
	import struct
	import timeit
	robot.send_msg(struct.pack('ff', hop_count_l, hop_count_r))# hove the robots send the messages with hop counts
	smooth_hop_l = [hop_count_l]#inclue own hop count
	smooth_hop_r = [hop_count_r]
	while timeit.default_timer() - timer < 10: # let the robot collect messages
		msgs = robot.recv_msg()
		if len(msgs) > 0:
			smoothed_hops = struct.unpack('ff', msgs[0][:8])
			smooth_hop_l.append(smoothed_hops[0])
			smooth_hop_r.append(smoothed_hops[1])

	smoothed_l_hop = (sum(smooth_hop_l)/len(smooth_hop_l))-.5  #Take averages as described in the paper
	smoothed_r_hop = (sum(smooth_hop_r)/len(smooth_hop_r))-.5
	#defining the lights to turn on 
	pos = get_cords(smoothed_l_hop,smoothed_r_hop)
	if pos[0]< (-.9):
		robot.set_led(100,100,100)
	elif pos[0]>.1:
		robot.set_led(100,100,100)
	elif pos[1]>(-1.2*pos[0]-.8) and pos[1]<(-1.2*pos[0]):
		robot.set_led(100,100,100)


