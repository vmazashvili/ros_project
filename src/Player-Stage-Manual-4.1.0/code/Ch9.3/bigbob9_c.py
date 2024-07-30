#/* bigbob9.py - demo ranger interface
# * K. Nickels 7/24/13
# */

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
sonarProxy = playerc_ranger(robot,0)
if sonarProxy.subscribe(PLAYERC_OPEN_MODE):
	raise Exception(playerc_error_str())
toothProxy = playerc_ranger(robot,1)
if toothProxy.subscribe(PLAYERC_OPEN_MODE):
	raise Exception(playerc_error_str())
laserProxy = playerc_ranger(robot,2)
if laserProxy.subscribe(PLAYERC_OPEN_MODE):
	raise Exception(playerc_error_str())

#  /* read from the proxies */
sonarProxy.get_geom()
toothProxy.get_geom()
laserProxy.get_geom()
robot.read()

print "max range = ", laserProxy.max_range

print "%d sonar ranges: "% sonarProxy.ranges_count
for i in range(sonarProxy.ranges_count):
	print "%.3f, " % sonarProxy.ranges[i],
print "."
print "%d tooth laser ranges: "% toothProxy.ranges_count
for i in range(toothProxy.ranges_count):
	print "%.3f, " % laserProxy.ranges[i],
print "."
print "%d laser ranges: "% laserProxy.ranges_count
for i in range(laserProxy.ranges_count):
	print "%.3f, " % laserProxy.ranges[i],
print "."

# Clean up 
sonarProxy.unsubscribe()
toothProxy.unsubscribe()
laserProxy.unsubscribe()
robot.disconnect()
