#include "inverse_kinematics.h"
#include "linear_algebra.h"
#include <math.h>

#define PI 3.14159265358979323846f

Mat3x3 InverseKinematics_CalculateJacobian(float theta0, float theta1, float theta2, float l1, float l2) {
	float sin_theta0 = sinf(theta0);
	float cos_theta0 = cosf(theta0);
	float sin_theta1 = sinf(theta1);
	float cos_theta1 = cosf(theta1);

	float theta2_prime = theta1 + theta2 - PI; // Combined angle for the second joint
	float cos_theta2_prime = cosf(theta2_prime);
	float sin_theta2_prime = sinf(theta2_prime);

	float r = l1 * cos_theta1 + l2 * cos_theta2_prime; // Effective length in the radial direction
	float dr_dtheta1 = -l1 * sin_theta1 - l2 * sin_theta2_prime; // Derivative of r with respect to theta1
	float dr_dtheta2 = -l2 * sin_theta2_prime; // Derivative of r with respect to theta2

	Mat3x3 jacobian;

	// Calculate the Jacobian matrix based on the angles
	// Row 1: ∂x/∂θ
	jacobian.data[0][0] = -sin_theta0 * r;
	jacobian.data[0][1] = dr_dtheta1 * cos_theta0;
	jacobian.data[0][2] = dr_dtheta2 * cos_theta0;

	// Row 2: ∂y/∂θ
	jacobian.data[1][0] = cos_theta0 * r;
	jacobian.data[1][1] = dr_dtheta1 * sin_theta0;
	jacobian.data[1][2] = dr_dtheta2 * sin_theta0;

	// Row 3: ∂z/∂θ
	jacobian.data[2][0] = 0.0f;
	jacobian.data[2][1] = r;
	jacobian.data[2][2] = l2 * cos_theta2_prime;

	return jacobian;
}

// Add lambda^2 * I to a 3x3 matrix
Mat3x3 InverseKinematics_AddDamping(Mat3x3 m, float lambda) {
    for (int i = 0; i < 3; ++i)
        m.data[i][i] += lambda * lambda;
    return m;
}

Vec3 InverseKinematics_ComputeJointVelocities(float theta0, float theta1, float theta2, float l1, float l2, Vec3 desired_tip_velocity) {
	/*
	Mat3x3 jacobian = InverseKinematics_CalculateJacobian(theta0, theta1, theta2, l1, l2);
	Mat3x3 inverse_of_jacobian = LinearAlgebra_3x3Inverse(jacobian);
	return LinearAlgebra_Multiply3x3WithVector(inverse_of_jacobian, desired_tip_velocity);
	*/
	Mat3x3 jacobian = InverseKinematics_CalculateJacobian(theta0, theta1, theta2, l1, l2);
	Mat3x3 transpose_of_jacobian = LinearAlgebra_Transpose3x3(jacobian);
	Mat3x3 jtj = LinearAlgebra_Multiply3x3(transpose_of_jacobian, jacobian);
	Mat3x3 damped_jtj = InverseKinematics_AddDamping(jtj, 25.0f); // Add damping to avoid singularities
	Mat3x3 inverse_of_damped_jtj = LinearAlgebra_3x3Inverse(damped_jtj);

	// Final multiplication: dq = (Jᵗ * inv(JJᵗ + λ²I)) * v
	Mat3x3 damped_pseudo_inverse = LinearAlgebra_Multiply3x3(inverse_of_damped_jtj, transpose_of_jacobian);
	return LinearAlgebra_Multiply3x3WithVector(damped_pseudo_inverse, desired_tip_velocity);
}
