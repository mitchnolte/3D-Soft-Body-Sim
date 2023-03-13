#ifndef VECTOR_H
#define VECTOR_H

#include <valarray>
#include <vector>

// Vector for physics calculations
typedef std::valarray<float> Vector;

// Renaming std::vector to List to avoid confusion
template <typename T>
using List = std::vector<T>;


float vecDot(const Vector& u, const Vector& v);
float vecNorm(const Vector& v);

#endif
