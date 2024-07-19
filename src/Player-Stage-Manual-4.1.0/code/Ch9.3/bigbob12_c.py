# bigbob12_c.py - demo simulationproxy interface (GetSetPose)
# Original author: K. Nickels 6/10/15

import math, sys, os
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
sys.path.append('/usr/local/lib/python2.7/site-packages/')
from playerc import *

# Make proxies for Client, position2d, simulation
robot = playerc_client(None, 'localhost', 6665)
if robot.connect():
	raise Exception(playerc_error_str())

p = playerc_position2d(robot,0)
if p.subscribe(PLAYERC_OPEN_MODE):
	raise Exception(playerc_error_str())

s = playerc_simulation(robot,0)
if s.subscribe(PLAYERC_OPEN_MODE):
	raise Exception(playerc_error_str())

# read from the proxies
robot.read()

# go forward
p.set_cmd_vel(0.0, 0.0, 40.0 * math.pi / 180.0, 1)

rtn,x,y,a = s.get_pose2d("puck1")
print "Puck1 is at Pose = %.1f,%.1f,%.1f)" % (x,y,a)

rtn,x,y,a = s.get_pose2d("puck3")
print "Puck3 is at Pose = %.1f,%.1f,%.1f)" % (x,y,a)

s.set_pose2d("puck1",4,5,0)
robot.read()
rtn,x,y,a = s.get_pose2d("puck1")
print  "Puck1 is at Pose = %.1f,%.1f,%.1f)" % (x,y,a)

for i in range(5):
	robot.read()
	rtn,x,y,a = s.get_pose2d("bob1")
	print "bob1 is at Pose = %.2f,%.2f,%.2f)" % (x,y,a)
	print "bob1.p2d at Pose = %.2f,%.2f,%.2f)" % (p.px,p.py,p.pa)
	
# Now stop
p.set_cmd_vel(0.0, 0.0, 0.0, 1)

# Clean up (l.unsubcribe is for laser sensor)
s.unsubscribe()
p.unsubscribe()
robot.disconnect()
