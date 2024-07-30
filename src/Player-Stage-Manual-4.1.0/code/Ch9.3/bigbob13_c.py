# bigbob13_c.py - demo simulationproxy interface (GetSetProperty)
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
if p.subscribe(PLAYERC_OPEN_MODE) !=0:
	raise Exception(playerc_error_str())

s = playerc_simulation(robot,0)
if s.subscribe(PLAYERC_OPEN_MODE) !=0:
	raise Exception(playerc_error_str())

# read from the proxies
robot.read()

# read from the proxies
robot.read()

# This doesn't work, exactly - something is retrieved, but it's nonsensical
puck1color = player_color_t()
s.get_property("puck1","color",puck1color,4*4)
print "puck1 Color before = ", puck1color.red,  puck1color.green, puck1color.blue, puck1color.alpha
puck2color = player_color_t()
s.get_property("puck2","color",puck2color,4*4)
print "puck2 Color before = ", puck2color.red,  puck2color.green, puck2color.blue, puck2color.alpha
puck3color = player_color_t()
s.get_property("puck3","color",puck3color,4*4)
print "puck3 Color before = ", puck3color.red,  puck3color.green, puck3color.blue, puck3color.alpha

# Note this doesn't work
# perhaps something mapping int to fload and back??
green = player_color_t()
green.red = 255;  green.green =  255; green.blue = 255;  green.alpha = 255;

s.set_property("puck1","color",green,4*4)
s.set_property("puck2","color",green,4*4)
s.set_property("puck3","color",green,4*4)
print "Set to ", green.red,  green.green, green.blue, green.alpha

s.get_property("puck1","color",puck1color,4*4)
print "puck1 Color after = ", puck1color.red,  puck1color.green, puck1color.blue, puck1color.alpha
s.get_property("puck2","color",puck2color,4*4)
print "puck2 Color after = ", puck2color.red,  puck2color.green, puck2color.blue, puck2color.alpha
s.get_property("puck3","color",puck3color,4*4)
print "puck3 Color after = ", puck3color.red,  puck3color.green, puck3color.blue, puck3color.alpha


# Clean up (l.unsubcribe is for laser sensor)
s.unsubscribe()
robot.disconnect()
