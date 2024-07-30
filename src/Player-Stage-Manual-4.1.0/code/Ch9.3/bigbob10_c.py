#/* bigbob10_c.py - demo blobfinder interface
# * K. Nickels 7/24/13
# */

import math, sys, os
sys.path.append('/usr/local/lib/python2.7/site-packages/')
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
from playerc import *

# Make proxies for Client, blobfinder
robot = playerc_client(None, 'localhost', 6665)
if robot.connect():
	raise Exception(playerc_error_str())

bf = playerc_blobfinder(robot,0);
if bf.subscribe(PLAYERC_OPEN_MODE):
	raise Exception(playerc_error_str())

if robot.read() == None:
	raise Exception(playerc_error_str())

print bf.blobs_count," blobs found"
for i in range(bf.blobs_count):
	blob = bf.blobs[i];  # indexing not working right either - more SWIG
	print 'BLOB %d, ' % i
        # Can't seem to access .color .x .y .top .left .right .bottom from
        # python.  
		# blob is a pointer to a C structure playerc_blob_t
		# Probably a SWIG problem...
	robot.read()

