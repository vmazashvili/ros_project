/* bigbob12.c - demo simulationproxy interface (GetSetPose)
 * K. Nickels 7/24/13
 */

#include <stdio.h>
#include <unistd.h>
#include <libplayerc/playerc.h>

int main(int argc, char *argv[]) {
  playerc_client_t *robot;
  playerc_position2d_t *pp;
  playerc_simulation_t *sp;

  /* Create a client and connect it to the server. */
  robot = playerc_client_create(NULL, "localhost", 6665);
  if (0 != playerc_client_connect(robot)) return -1;

  /* Create and subscribe to a simulation proxy. */
  sp = playerc_simulation_create(robot, 0);
  if (playerc_simulation_subscribe(sp, PLAYER_OPEN_MODE)) return -1;

  pp = playerc_position2d_create(robot, 0);
  if (playerc_position2d_subscribe(pp, PLAYER_OPEN_MODE)) return -1;

  /* read from the proxies */
  playerc_client_read(robot);

  /* go forward */
  playerc_position2d_set_cmd_vel(pp, 0.1, 0.1, 0, 1);

  double x,y,a;
  playerc_simulation_get_pose2d(sp,(char *)"puck1",&x,&y,&a);
  printf("Puck1 is at Pose = %.1f,%.1f,%.1f)\n",x,y,a);

  playerc_simulation_get_pose2d(sp,(char *)"puck3",&x,&y,&a);
  printf("Puck3 is at Pose = %.1f,%.1f,%.1f)\n",x,y,a);

  playerc_simulation_set_pose2d(sp,(char *)"puck1",4,5,0);
  playerc_client_read(robot);
  playerc_simulation_get_pose2d(sp,(char *)"puck1",&x,&y,&a);
  printf("Puck1 is at Pose = %.1f,%.1f,%.1f)\n",x,y,a);

  for(int i=0;i<5;i++) {
		playerc_client_read(robot);
		playerc_simulation_get_pose2d(sp,(char *)"bob1",&x,&y,&a);
  		printf("bob1 is at Pose = %.2f,%.2f,%.2f)\n",x,y,a);
    }

  playerc_position2d_set_cmd_vel(pp, 0.0, 0.0, 0, 1);

  /* Shutdown */
  playerc_simulation_unsubscribe(sp);
  playerc_position2d_unsubscribe(pp);
  playerc_simulation_destroy(sp);
  playerc_position2d_destroy(pp);
  playerc_client_disconnect(robot);
  playerc_client_destroy(robot);

  return 0;
}

