# Bigbob8.py - GetSetPositions

# Check with "locate playerc.py"
import sys,os
sys.path.append('/usr/local/lib/python2.7/site-packages/')
sys.path.append('/usr/local/lib64/python2.7/site-packages/')

import math
from playerc import *

#  /* Create a client and connect it to the server. */
robot = playerc_client(None, 'localhost',6665)
if robot.connect():
	raise Exception(playerc_error_str())

#  /* Create and subscribe to a position2d device. */
p2dProxy = playerc_position2d(robot,0)
if p2dProxy.subscribe(PLAYERC_OPEN_MODE):
	raise Exception(playerc_error_str())

#  /* read from the proxies */
robot.read()

p2dProxy.enable(True)

#  /* for carlike robots ( XSpeed(m/s), steering_angle (radians) )
#  //  playerc_position2d_set_cmd_car (P2dProxy, 0.2,dtor(30)); 
#  // for holonomic robots (XSpeed (m/s), YSpeed (m/s), YawSpeed (rad/s), state)
#  // *You* should check return values!!
p2dProxy.set_cmd_vel(0.1, 0.0, math.radians(40.0), 1);

for i in range(10):
	# Wait for new data from server
	if robot.read() == None:
		raise Exception(playerc_error_str())
	print "Iter %d"% i
	print "XSpeed   = %.2f m/s\t"% p2dProxy.vx,
	print "YSpeed   = %.2f m/s\t"% p2dProxy.vy,
	print "YawSpeed = %.2f rad/s"% p2dProxy.va
	print "XPos     = %.2f m\t"% p2dProxy.px,
	print "YPos     = %.2f m\t"% p2dProxy.py,
	print "Yaw      = %.2f rad"% p2dProxy.pa
	
# Now stop
p2dProxy.set_cmd_vel(0.0, 0.0, 0.0, 1)

# Clean up (l.unsubcribe is for laser sensor)
p2dProxy.unsubscribe()
robot.disconnect()
