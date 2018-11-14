
#ifndef COMMONDEFINES_H
#define COMMONDEFINES_H

// maximum level of pyramid processing in registration
#define MAX_LEVELS	6


// the maximum length of the tracking trajectory
// it is implemented as a ring buffer type of structure
#define MAX_TRAJECTORY_LENGTH			1000
// the maximum length of the tracking target list
#define MAX_TARGET_LIST_LENGTH			50

// the type of motion that is supported in this package
typedef enum {
  DEFAULT=0,
  MOTION_TRANSLATION,
  MOTION_SCALE_ROTATION,
  MOTION_AFFINE
} MOTION_TYPE;

// 2D position on the image plane
struct Position2Di {
	int x,y;
};

// general information regarding a target being tracked
struct TargetInfo {
	int xstart, ystart, xsize, ysize;	// location and size
	int mean;							// mean value
	long size;							// area

	Position2Di trajectory[MAX_TRAJECTORY_LENGTH];	// list of the points along the trajectory
	int traj_length;								// current length of the trajectory
};

#endif
