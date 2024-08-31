/*
A simulation of a simple robotic system that operates in a grid-based
world. The robot is represented by a unicycle platform and it has a 
laser scanner for detecting objects in its environment. The program uses the Eigen library for matrix operations and transformations. 
*/

#include <iostream>
#include "grid_map.h"
#include "world_item.h"
#include "laser_scanner.h"

using namespace std; // Use the standard namespace

// Function to create a 2D Isometry transformation from translation and rotation coefficients
Isometry2f fromCoefficients(float tx, float ty, float alpha) {
  Isometry2f iso;
  iso.setIdentity();								// Initialize as identity transformation (no transformation)
  iso.translation()<< tx, ty;						// Set translation component (tx, ty)
  iso.linear()=Eigen::Rotation2Df(alpha).matrix();	// Set rotation component (rotation by angle alpha)
  return iso;										// Return the resulting transformation
}

int main(int argc, char** argv) {
  // Check if the program has received the correct number of arguments
  if (argc < 2) {
    cout << "usage: " << argv[0] << " <image_file> <resolution>" << endl;
    return -1;
  }
  
  // Parse command-line arguments
  const char* filename = argv[1];	// First argument: image filename
  float resolution = atof(argv[2]);	// Second argument: resolution of the grid map

  // Output the parsed arguments
  cout << "Running " << argv[0] << " with arguments" << endl
       << "-filename:" << argv[1] << endl
       << "-resolution: " << argv[2] << endl;

  // Create a GridMap object with initial size (0, 0) and resolution 0.1
  GridMap grid_map(0, 0, 0.1);
  grid_map.loadFromImage(filename, resolution);

  // Create a world object using the grid map
  World world_object(grid_map);
  
  // Create a WorldItem object at a specific location (5, 0) with a rotation of 0.5 radians
  WorldItem object_0(world_object, fromCoefficients(5, 0, 0.5));
  
  // Calculate the middle of the grid in grid coordinates
  Vector2f grid_middle(grid_map.cols/2, grid_map.rows/2);
  
  // Convert grid coordinates to world coordinates
  Vector2f world_middle = grid_map.grid2world(grid_middle);
  
  // Create a UnicyclePlatform (robot) at the center of the world, with an initial rotation of -0.5 radians
  UnicyclePlatform robot(world_object, fromCoefficients(world_middle.x(), world_middle.y(), -0.5));
  robot.radius=1;

  // Initialize a LaserScan object (to store scan data)
  LaserScan scan;
  
  // Create a LaserScanner attached to the robot at an offset (3, 0) with no additional rotation
  LaserScanner scanner(scan, robot, fromCoefficients(3, 0, -0));
  scanner.radius = 0.5;


  // Time step for the simulation
  float dt=0.1;
  
  // Canvas object for drawing the simulation
  Canvas canvas;
  
  // Main simulation loop
  while (true) {
    world_object.tick(dt);			// Update the world state based on the time step
    world_object.draw(canvas);		// Draw the current state of the world on the canvas
    int ret = showCanvas(canvas, dt*100);	// Display the canvas, wait for user input
    
    // Check if a key has been pressed 
    if (ret>0)
      std::cerr << "Key pressed: " << ret << std::endl;

	// Handle user input (control the robot)
    switch(ret) {
    case 81: // Left arrow key
      robot.rv+=0.1;	// Increase rotational velocity (turn left)
      break;
    case 82: // Up arrow key
      robot.tv+=0.1;	// Increase translational velocity (move forward)
      break;
    case 83: // Right arrow key
      robot.rv-=0.1;	// Decrease rotational velocity (turn right)
      break;
    case 84: // Down arrow key
      robot.tv-=0.1;	// Decrease translational velocity (move backward)
      break;
    case 32: // Space bar
      robot.rv=0;	// Stop rotational movement
      robot.tv=0;	// Stop translational movement
      break;
    default:;	// Do nothing for other keys
    }
    
  }
}
