#ifndef ARM_CONFIGURATION_H
#define ARM_CONFIGURATION_H

#include "linear_algebra.h"

#define LENGTH_OF_FIRST_ARM 206.0f // Length of the first arm segment in mm//206
#define LENGTH_OF_SECOND_ARM 330.0f // Length of the second arm segment in mm//  335

typedef struct
{
	float radius;
	Vec3 center_position;
	float viscous_thickness;
} SphericalVolume;

static const float JOINT_HOME_ANGLES[] = {87.5f, 172.5f, -4.2f, 0.0f}; // Home angles for each joint in degrees
static const float JOINT_LIMIT_OFFSETS[] = {3.0f, 3.0f, 3.0f}; // Joint limits offsets in degrees
static const float JOINT_DIRECTIONS[] = {1, -1, 1, 1}; // Home angles for each joint in degrees
static const SphericalVolume BLOCKED_SPHERICAL_VOLUMES[3] = {
	{
		.radius = 85.0f,
		.center_position = { {0.0f, 0.0f, 0.0f} },
		.viscous_thickness = 30.0f
	},
	/*
	{
		.radius = 300.0f,


		.center_position = { {0.0f, 0.0f, 0.0f} },
		.viscous_thickness = 8.0f
	},
	{
		.radius = 400.0f,
		.center_position = { {0.0f, 0.0f, 0.0f} },
		.viscous_thickness = 12.0f
	}
	*/
};

#define NUMBER_OF_BLOCKED_SPHERES (sizeof(BLOCKED_SPHERICAL_VOLUMES) / sizeof(BLOCKED_SPHERICAL_VOLUMES[0]))


#endif
