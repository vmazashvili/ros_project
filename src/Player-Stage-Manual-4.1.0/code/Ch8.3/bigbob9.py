#/* bigbob9.c - demo ranger interface
# * K. Nickels 6/5/14
# */

# Import all of this for all entries.  Fix path as necessary
import sys, os, math
sys.path.append('/usr/local/lib/python2.7/site-packages/')
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
from playercpp import *

# Create proxies for Client, Sonar, Tooth, Laser, Position2d
robot = PlayerClient("localhost");
sonarProxy = RangerProxy(robot,0);
toothProxy = RangerProxy(robot,1);
laserProxy = RangerProxy(robot,2);

# Request Geometry
sonarProxy.RequestGeom(); 
sonarProxy.RequestConfigure(); # fills up angle structures
toothProxy.RequestGeom();
toothProxy.RequestConfigure();
laserProxy.RequestGeom();
laserProxy.RequestConfigure();

# Read from the proxies
robot.Read()

# Request Geometry from the rest of the proxies
# doesn't work
#print "%d Sonar Ranges (method 1): " % sonarProxy.GetRangeCount()
#for i in range(sonarProxy.GetRangeCount()):
#	print '%.3f ' % sonarProxy[i],
#print '.'

# Method 2, same
print "%d Sonar Ranges: " % sonarProxy.GetRangeCount()
for i in range(sonarProxy.GetRangeCount()):
	print '%.1f ' % sonarProxy.GetRange(i),
print '.'

print "%d tooth ranges: " % toothProxy.GetRangeCount()
for i in range(toothProxy.GetRangeCount()):
	print '%.1f ' % toothProxy.GetRange(i),
print '.'
	
print "%d laser ranges: "% laserProxy.GetRangeCount()
for i in range(laserProxy.GetRangeCount()):
	print '%.1f ' % laserProxy.GetRange(i),
print '.'
