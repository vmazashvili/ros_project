# bigbob2a.py
# Case Study 2 - multiple robots (method 1)

import math, sys, os
sys.path.append('/usr/local/lib/python2.7/site-packages/')
from playercpp import *

# Make proxies for Client, Sonar, Position2d

# first robot
robot = PlayerClient("localhost",6665);
sp = RangerProxy(robot,0);
lp = RangerProxy(robot,1);
pp = Position2dProxy(robot,0);

# second robot
robot2 = PlayerClient("localhost",6666);
sp2 = RangerProxy(robot2,0);
lp2 = RangerProxy(robot2,1);
pp2 = Position2dProxy(robot2,0);

while True:

	# Read from proxies
	robot.Read()

	# Print out sonars for fun
	print "Sonar scan (robot 1, %d ranges): " % sp.GetRangeCount(),
	for i in range(sp.GetRangeCount()):
		print '%.2f' % sp.GetRange(i),
	print '.'
	# Print out lasers for fun
	print "Laser scan (robot 1, %d ranges): " % lp.GetRangeCount(),
	for i in range(lp.GetRangeCount()):
		print '%.2f' % lp.GetRange(i),
	print '.'

	# do simple collision avoidance
	short = 0.5;
	if sp.GetRange(0) < short or sp.GetRange(2)<short:
		turnrate = math.radians(-20); # Turn 20 degrees persecond
	elif sp.GetRange(1) <short or sp.GetRange(3)<short:
		turnrate = math.radians(20)
	else:
		turnrate = 0;

	if sp.GetRange(0) < short or sp.GetRange(1) < short:
		speed = 0;
	else:
		speed = 0.100;

	# Command the motors
	pp.SetSpeed(speed, turnrate);


	# Read from proxies
	robot2.Read()

	# Print out sonars for fun
	print "Sonar scan (robot 2), %d ranges: " % sp2.GetRangeCount(),
	for i in range(sp2.GetRangeCount()):
		print '%.2f' % sp2.GetRange(i),
	print '.'
	# Print out lasers for fun
	print "Laser scan (robot 2), %d ranges: " % lp2.GetRangeCount(),
	for i in range(lp2.GetRangeCount()):
		print '%.2f' % lp2.GetRange(i),
	print '.'

	# do simple collision avoidance
	short = 0.5;
	if sp2.GetRange(0) < short or sp2.GetRange(2)<short:
		turnrate = math.radians(-20); # Turn 20 degrees persecond
	elif sp2.GetRange(1) <short or sp2.GetRange(3)<short:
		turnrate = math.radians(20)
	else:
		turnrate = 0;

	if sp2.GetRange(0) < short or sp2.GetRange(1) < short:
		speed = 0;
	else:
		speed = 0.100;

	# Command the motors
	pp2.SetSpeed(speed, turnrate);

