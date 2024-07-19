# bigbob12.py - demo simulationproxy interface (GetSetPose)
# Original author: K. Nickels 6/10/15

import sys, os, math
sys.path.append('/usr/local/lib/python2.7/site-packages/')
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
from playercpp import *

robot = PlayerClient("localhost")
p2dProxy = Position2dProxy(robot)
sp = SimulationProxy(robot)

# Read from the proxies
robot.Read()

# Print out stuff for fun
print sp
print p2dProxy


# command the motors
p2dProxy.SetSpeed(0.1,0.1); # XSpeed (m/s), YawSpeed (rad/s)
robot.Read()

#sp.GetPose2d("puck1",px,py,pa)
#print "Puck1 is at Pose = ",x,", ",y,", ",a

#sp.GetPose2d("puck3",px,py,pa);
#print "Puck3 is at Pose = ",x,", ",y,", ",a

sp.SetPose2d("puck1",4,5,0);
robot.Read();
#sp.GetPose2d("puck1",px,py,pa);
#print "Puck1 is now at Pose = ",x,", ",y,", ",a

for i in range(5):
	robot.Read()
	#sp.GetPose2d("bob1",px,py,pa)
	#print "bob1 Pose = %.2f, %.2f, %.2f" % (x,y,a)
	# This gives position2d (odometry) rather than ground truth.
	# may be different if you have odom_error set.
	print "bob1 Pose = %.2f, %.2f, %.2f" % (p2dProxy.GetXPos(),p2dProxy.GetYPos(),p2dProxy.GetYaw())


# Stop
p2dProxy.SetSpeed(0,0)
