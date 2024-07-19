# fids.py - demonstrate use of fiducials
# Original author: K. Nickels 6/10/15

import sys, os, math
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
sys.path.append('/usr/local/lib/python2.7/site-packages/')
from playercpp import *

robot = PlayerClient("localhost")
fidsProxy = FiducialProxy(robot)

# Read from the proxies
robot.Read()

# Print out stuff for fun
print fidsProxy

robot.Read()
print "%d fiducials found" % fidsProxy.GetCount()

for i in range(fidsProxy.GetCount()):
	robot.Read()
	item = fidsProxy.GetFiducialItem(i)
# same problem as blobfinder - can't get to stuff in player_fiducial_item
# struct!!!
	print item
