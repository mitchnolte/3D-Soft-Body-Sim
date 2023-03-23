#ifndef VECTOR_H
#define VECTOR_H

#include <valarray>
#include <vector>

// Vector for physics calculations
typedef std::valarray<double> Vector;
typedef std::vector<Vector> VecList;


double vecDot(const Vector& u, const Vector& v);
double vecNorm(const Vector& v);
Vector normalize(const Vector& v);

VecList operator+(VecList l1, VecList l2);
VecList operator+(double scalar, VecList list);
VecList operator+(VecList list, double scalar);
VecList operator*(VecList l1, VecList l2);
VecList operator*(double scalar, VecList list);
VecList operator*(VecList list, double scalar);

#endif
