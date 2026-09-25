#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "linear_algebra.h"

#define PI 3.14159265358979323846f
#define POSITION_CONTROL_RADIUS 10.0f

void ArmController_ApplyJointVelocity(uint8_t joint_index, int32_t velocity);
void ArmController_ApplyIKVelocity(Vec3 tip_velocity);
void ArmController_StopAllJoints(void);
Vec3 ArmController_ModifyTipVelocity(float theta0, float theta1, float theta2, Vec3 desired_tip_velocity);
Vec3 ArmController_CalculateTipPosition(float theta0, float theta1, float theta2);
bool ArmController_CheckIfJointMotionAllowed(float joint, float joint_velocity, uint8_t index);
bool ArmController_MoveTipToPosition(Vec3 target_position, float speed, bool will_stop_when_arrived);
void ArmController_DriveCartesianClosedLoop(Vec3 tip_velocity, float dt);
#endif
