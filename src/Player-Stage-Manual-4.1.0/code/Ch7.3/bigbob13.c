/* bigbob13.cc - demo simulationproxy interface (GetSetProperty)
 * K. Nickels 7/2/15
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

    // get and re-set the color
    float green[]= {0.67, 0.88, 0.43, 1};
    float puckcolor[4];
  	playerc_client_read(robot);

	playerc_simulation_get_property(sp,(char *)"puck1",(char*)"color",puckcolor,4*sizeof(float));
	printf("Puck1 is color = (%.2f,%.2f,%.2f,%.2f)\n",
			puckcolor[0], puckcolor[1], puckcolor[2], puckcolor[3]);

	playerc_simulation_get_property(sp,(char *)"puck2",(char*)"color",puckcolor,4*sizeof(float));
	printf("Puck2 is color = (%.2f,%.2f,%.2f,%.2f)\n",
			puckcolor[0], puckcolor[1], puckcolor[2], puckcolor[3]);

	playerc_simulation_get_property(sp,(char *)"puck3",(char*)"color",puckcolor,4*sizeof(float));
	printf("Puck3 is color = (%.2f,%.2f,%.2f,%.2f)\n",
			puckcolor[0], puckcolor[1], puckcolor[2], puckcolor[3]);

    printf("setting puck1 to green\n");

	playerc_simulation_set_property(sp,(char *)"puck1",(char*)"color",green,4*sizeof(float));

	playerc_simulation_get_property(sp,(char *)"puck1",(char*)"color",puckcolor,4*sizeof(float));
	printf("Puck3 is color = (%.2f,%.2f,%.2f,%.2f)\n",
			puckcolor[0], puckcolor[1], puckcolor[2], puckcolor[3]);

  /* Shutdown */
  playerc_simulation_unsubscribe(sp);
  playerc_position2d_unsubscribe(pp);
  playerc_simulation_destroy(sp);
  playerc_position2d_destroy(pp);
  playerc_client_disconnect(robot);
  playerc_client_destroy(robot);

  return 0;
}

