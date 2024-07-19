# bigbob13.py - demo simulationproxy interface (GetSetProperty)
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

robot.Read()

# This doesn't work, exactly - something is retrieved, but it's nonsensical
puck1color = player_color_t()
sp.GetProperty("puck1","color",puck1color,4*4)
print "puck1 Color before = ", puck1color.red,  puck1color.green, puck1color.blue, puck1color.alpha
puck2color = player_color_t()
sp.GetProperty("puck2","color",puck2color,4*4)
print "puck2 Color before = ", puck2color.red,  puck2color.green, puck2color.blue, puck2color.alpha
puck3color = player_color_t()
sp.GetProperty("puck3","color",puck3color,4*4)
print "puck3 Color before = ", puck3color.red,  puck3color.green, puck3color.blue, puck3color.alpha

# Note this doesn't work
# perhaps something mapping int to fload and back??
green = player_color_t()
green.red = 255;  green.green =  255; green.blue = 255;  green.alpha = 255;

sp.SetProperty("puck1","color",green,4*4)
sp.SetProperty("puck2","color",green,4*4)
sp.SetProperty("puck3","color",green,4*4)
print "Set to ", green.red,  green.green, green.blue, green.alpha

sp.GetProperty("puck1","color",puck1color,4*4)
print "puck1 Color after = ", puck1color.red,  puck1color.green, puck1color.blue, puck1color.alpha
sp.GetProperty("puck2","color",puck2color,4*4)
print "puck2 Color after = ", puck2color.red,  puck2color.green, puck2color.blue, puck2color.alpha
sp.GetProperty("puck3","color",puck3color,4*4)
print "puck3 Color after = ", puck3color.red,  puck3color.green, puck3color.blue, puck3color.alpha

