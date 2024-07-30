# bigbob8.py - demo GetSpeed GetPos
# Original author: K. Nickels 7/24/13

import sys, os, math
# This should be wherever "playercpp.py" is
sys.path.append('/usr/local/lib/python2.7/site-packages/')
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
from playercpp import *


# read from the proxies
robot = PlayerClient("localhost");
pp = Position2dProxy(robot,0);


# for carlike robots
# p2dProxy.SetSpeed(0.1,0.1); # XSpeed (m/s), YawSpeed (rad/s)
pp.SetCarlike(0.2,math.radians(3.0)); # XSpeed, Yaw (rad)
# for holonomic robots
# pp.SetSpeed(0.1,0.1,0.1); # XSpeed (m/s), YawSpeed (rad/s)

for i in range(10):
	robot.Read()
	print 'Iter ',i
	print 'XSpeed = %.3f m/s, ' % pp.GetXSpeed(), 
	print 'YSpeed = %.3f m/s, ' % pp.GetYSpeed(), 
	print 'YawSpeed = %.3f rad/s ' % pp.GetYawSpeed()
	print 'XPos = %.3f m, ' % pp.GetXPos(),
	print 'YPos = %.3f m, ' % pp.GetYPos(),
	print 'Yaw = %.3f rad' % pp.GetYaw() 

# Destructors called automatically on exit.
