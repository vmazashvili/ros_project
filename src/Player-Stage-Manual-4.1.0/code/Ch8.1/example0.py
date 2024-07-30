# example0.py - a conversion of example file example0.cc distributed with
# player-3.0.2
# Converted K. Nickels

import math, sys, os
sys.path.append('/usr/local/lib/python2.7/site-packages/')
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
from playercpp import *

# Make proxies for Client, Sonar, Position2d
robot = PlayerClient("localhost");
sp = RangerProxy(robot,0);
pp = Position2dProxy(robot,0);

while True:

	# Read from proxies
	robot.Read()

	# Print out sonars for fun
	print "Sonar scan: ",
	for i in range(sp.GetRangeCount()):
		print '%.2f' % sp.GetRange(i),
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
