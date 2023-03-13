#include "vector.h"

float vecDot(const Vector& u, const Vector& v) {
  return (u * v).sum();
}

float vecNorm(const Vector& v) {
  return sqrt(vecDot(v, v));
}
