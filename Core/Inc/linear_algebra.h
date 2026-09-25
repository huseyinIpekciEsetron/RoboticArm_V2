// linear_algebra.h
#ifndef LINEAR_ALGEBRA_H
#define LINEAR_ALGEBRA_H

typedef struct {
	float data[3][3];
} Mat3x3;

typedef struct {
	float data[3];
} Vec3;

float LinearAlgebra_3x3Determinant(Mat3x3 m);
Mat3x3 LinearAlgebra_3x3Inverse(Mat3x3 m);
Mat3x3 LinearAlgebra_Transpose3x3(Mat3x3 m);
Vec3 LinearAlgebra_Multiply3x3WithVector(Mat3x3 m, Vec3 v);
Mat3x3 LinearAlgebra_Multiply3x3(Mat3x3 a, Mat3x3 b);
Vec3 LinearAlgebra_Subtract(Vec3 a, Vec3 b);
Vec3 LinearAlgebra_Scale(Vec3 v, float s);
float LinearAlgebra_CalculateMagnitude(Vec3 v);
Vec3 LinearAlgebra_Normalize(Vec3 v);
float LinearAlgebra_CalculateDotProduct(Vec3 a, Vec3 b);
Vec3 LinearAlgebra_ProjectOnPlane(Vec3 v, Vec3 normal);

#endif