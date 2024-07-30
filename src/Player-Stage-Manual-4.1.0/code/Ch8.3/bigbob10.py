#/* bigbob10.py - demo blobfinder interface
# * K. Nickels 7/24/13
# */

import math, sys, os
sys.path.append('/usr/local/lib/python2.7/site-packages/')
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
from playerc import *
from playercpp import *

# Make proxies for Client, blobfinder
robot = PlayerClient("localhost");
bf = BlobfinderProxy(robot,0);

robot.Read() # fills up bf the first time

print bf.GetCount()," blobs found"
for i in range(bf.GetCount()):
	blob = bf.GetBlob(i);
	dir(blob)
	print 'BLOB %d, ' % i
	print blob.__getattr__("x")
        # Can't seem to access .color .x .y .top .left .right .bottom from
        # python.  
		# blob is a pointer to a C structure playerc_blob_t
		# Probably a SWIG problem...

	robot.Read()

