#/* bigbob11.cc 
# * Shows gripper, and how it works.
# * K. Nickels 7/24/13
# */

import sys, os
sys.path.append('/usr/local/lib/python2.7/site-packages/')
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
from playerc import *

import math,time

def gripper_state (state): 
    return {
        PLAYER_GRIPPER_STATE_OPEN: 'open',
        PLAYER_GRIPPER_STATE_CLOSED: 'closed',
        PLAYER_GRIPPER_STATE_MOVING: 'moving',
        PLAYER_GRIPPER_STATE_ERROR: 'error',
        }.get(state,'unknown')


# Make proxies for Client, Gripper, and Position2d
robot = playerc_client(None, 'localhost', 6665)
if robot.connect():
	raise Exception(playerc_error_str())
gp = playerc_gripper(robot,0)
if gp.subscribe(PLAYERC_OPEN_MODE):
	raise Exception(playerc_error_str())
p = playerc_position2d(robot,0)
if p.subscribe(PLAYERC_OPEN_MODE):
	raise Exception(playerc_error_str())

#  /* read from the proxies */
gp.get_geom()
robot.read()
gp.printout("state of gripper");
print "Number of breakbeams: %d" % gp.num_beams

# start moving
p.set_cmd_vel(0.1, 0.0, 0.0, 1)

# Keep going till you see something
while(not gp.beams):
	robot.read()
	print "Gripper is", gripper_state(gp.state)
	while (not gp.beams):
		robot.read()
		# print "%d " % gp.beams
	print "."
	print "Arrived at object."
	print "Gripper is", gripper_state(gp.state)

#  /* stop and close gripper */
print  "Stop and close gripper..."
print  "Gripper is ", gripper_state(gp.state)
p.set_cmd_vel(0.0, 0.0, 0, 1)
gp.close_cmd()
while (gp.state != PLAYER_GRIPPER_STATE_CLOSED):
	robot.read()
	# print  "Gripper is ", gripper_state(gp.state)

print  "Gripper is ", gripper_state(gp.state)

#  /* Note - in stage there is a strange bug drawing the paddles on closing
#   * the first time.
#   */

#  /* drive around with your box for a while */
p.set_cmd_vel(-0.1, 0.0, math.radians(30), 1)
time.sleep(2)

#  /* Now drop the box and speed up */
p.set_cmd_vel(-0.5, 0.0, 0, 1)
gp.open_cmd()
time.sleep(2)

# Now stop
p.set_cmd_vel(0.0, 0.0, 0.0, 1)

#  /* Shutdown */
gp.unsubscribe()
p.unsubscribe()
gp.destroy()
p.destroy()
robot.disconnect()
robot.destroy()
