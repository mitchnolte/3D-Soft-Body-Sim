#ifndef VECTOR_H
#define VECTOR_H

#include <valarray>
#include <vector>

// Vector for physics calculations
typedef std::valarray<float> Vector;
typedef std::vector<Vector> VecList;


float vecDot(const Vector& u, const Vector& v);
float vecNorm(const Vector& v);

VecList operator+(VecList l1, VecList l2);
VecList operator*(VecList l1, VecList l2);
VecList operator*(float scalar, VecList list);
VecList operator*(VecList list, float scalar);

#endif
