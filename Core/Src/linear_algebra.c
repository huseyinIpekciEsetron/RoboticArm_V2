#include "linear_algebra.h"
#include <math.h>

float LinearAlgebra_3x3Determinant(Mat3x3 m) {
  float a = m.data[0][0], b = m.data[0][1], c = m.data[0][2];
  float d = m.data[1][0], e = m.data[1][1], f = m.data[1][2];
  float g = m.data[2][0], h = m.data[2][1], i = m.data[2][2];

  return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

Mat3x3 LinearAlgebra_3x3Inverse(Mat3x3 m) {
  Mat3x3 inversed;
  float determinant = LinearAlgebra_3x3Determinant(m);

  if (fabsf(determinant) < 1e-6f) {
      // Handle singular case (return identity or error)
      for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++)
          inversed.data[i][j] = (i == j) ? 1.0f : 0.0f;
      return inversed;
  }

  float a = m.data[0][0], b = m.data[0][1], c = m.data[0][2];
  float d = m.data[1][0], e = m.data[1][1], f = m.data[1][2];
  float g = m.data[2][0], h = m.data[2][1], i = m.data[2][2];

  inversed.data[0][0] =  (e * i - f * h);
  inversed.data[0][1] = -(b * i - c * h);
  inversed.data[0][2] =  (b * f - c * e);

  inversed.data[1][0] = -(d * i - f * g);
  inversed.data[1][1] =  (a * i - c * g);
  inversed.data[1][2] = -(a * f - c * d);

  inversed.data[2][0] =  (d * h - e * g);
  inversed.data[2][1] = -(a * h - b * g);
  inversed.data[2][2] =  (a * e - b * d);

  // Transpose the cofactor matrix and divide by determinant
  for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
          inversed.data[i][j] /= determinant;

  return inversed;
}

Mat3x3 LinearAlgebra_Transpose3x3(Mat3x3 m) {
    Mat3x3 t;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            t.data[i][j] = m.data[j][i];
    return t;
}

Vec3 LinearAlgebra_Multiply3x3WithVector(Mat3x3 m, Vec3 v) {
  Vec3 result;
  for (int i = 0; i < 3; ++i) {
    result.data[i] = 0.0f;
    for (int j = 0; j < 3; ++j) {
      result.data[i] += m.data[i][j] * v.data[j];
    }
  }
  return result;
}

Mat3x3 LinearAlgebra_Multiply3x3(Mat3x3 a, Mat3x3 b) {
    Mat3x3 result = {0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                result.data[i][j] += a.data[i][k] * b.data[k][j];
    return result;
}

Vec3 LinearAlgebra_Subtract(Vec3 a, Vec3 b) {
	Vec3 result;
	for (int i = 0; i < 3; ++i) {
		result.data[i] = a.data[i] - b.data[i];
	}
	return result;
}

Vec3 LinearAlgebra_Scale(Vec3 v, float s) {
	Vec3 result;
	for (int i = 0; i < 3; ++i) {
		result.data[i] = v.data[i] * s;
	}
	return result;
}

float LinearAlgebra_CalculateMagnitude(Vec3 v) {
	return sqrtf(v.data[0] * v.data[0] + v.data[1] * v.data[1] + v.data[2] * v.data[2]);
}

Vec3 LinearAlgebra_Normalize(Vec3 v) {
	float magnitude = LinearAlgebra_CalculateMagnitude(v);
	Vec3 result;
	if (magnitude < 1e-6f) {
		// Handle zero vector case
		result.data[0] = 0.0f;
		result.data[1] = 0.0f;
		result.data[2] = 0.0f;
	} else {
		result = LinearAlgebra_Scale(v, 1.0f / magnitude);
	}
	return result;
}

float LinearAlgebra_CalculateDotProduct(Vec3 a, Vec3 b) {
	return a.data[0] * b.data[0] + a.data[1] * b.data[1] + a.data[2] * b.data[2];
}

Vec3 LinearAlgebra_ProjectOnPlane(Vec3 v, Vec3 normal) {
    // Assumes 'normal' is a unit vector (normalized)
    float dot_product = LinearAlgebra_CalculateDotProduct(v, normal);
    Vec3 projection = LinearAlgebra_Subtract(v, LinearAlgebra_Scale(normal, dot_product));
    return projection;
}
