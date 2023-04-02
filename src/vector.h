#ifndef VECTOR_H
#define VECTOR_H

#include <valarray>
#include <vector>
#include <glm/glm.hpp>

// Types
typedef std::valarray<double>     Vector;    // Vector for physics calculations
typedef std::vector<Vector>       VecList;   // List of physics vectors


// Vector operations
double vecDot(const Vector& u, const Vector& v);
double vecNorm(const Vector& v);
Vector normalize(const Vector& v);
Vector vecCross(const Vector& u, const Vector& v);
Vector matVecMul(const glm::mat4& M, const Vector v, float w=0.0f);
Vector planeNormal(const Vector& p1, const Vector& p2, const Vector& p3);

// Element-wise operators for lists of vectors
VecList operator+(VecList l1, VecList l2);
VecList operator+(double scalar, VecList list);
VecList operator+(VecList list, double scalar);
VecList operator*(VecList l1, VecList l2);
VecList operator*(double scalar, VecList list);
VecList operator*(VecList list, double scalar);

#endif
