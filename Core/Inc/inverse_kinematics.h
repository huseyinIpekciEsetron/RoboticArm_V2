// inverse_kinematics.h
#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include "linear_algebra.h"


Mat3x3 InverseKinematics_AddDamping(Mat3x3 m, float lambda);
Mat3x3 InverseKinematics_CalculateJacobian(float theta0, float theta1, float theta2, float l1, float l2);
Vec3 InverseKinematics_ComputeJointVelocities(float theta1, float theta2, float theta3, float l1, float l2, Vec3 desired_tip_velocity);

#endif
