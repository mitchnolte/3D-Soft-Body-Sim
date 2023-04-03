#include "vector.h"


double vecDot(const Vector& u, const Vector& v)   {return (u * v).sum();}
double vecNorm(const Vector& v)                   {return sqrt(vecDot(v, v));}
Vector normalize(const Vector& v)                 {return v / vecNorm(v);}
Vector vecCross(const Vector& u, const Vector& v) {
  return Vector{u[1]*v[2] - u[2]*v[1],  u[2]*v[0] - u[0]*v[2],  u[0]*v[1] - u[1]*v[0]};
}

/**
 * @brief  Matrix-vector multiplication. 
 *
 * @param  M  4x4 matrix.
 * @param  v  Vector with 3 or 4 elements.
 * @param  w  The 4th element of the vector used in the multiplication if the
 *            given vector is of length 3. Defaults to 0 and isn't used if the
 *            given vector already has 4 elements.
 *
 * @return Result of multiplication or the original vector if it has less than 3
 *         or greater than 4 elements.
 */
Vector matVecMul(const glm::mat4& M, const Vector v, float w) {
  if(v.size() == 3) {
    glm::vec4 result = M * glm::vec4(v[0], v[1], v[2], w);
    return Vector{result.x, result.y, result.z};
  }
  else if(v.size() == 4) {
    glm::vec4 result = M * glm::vec4(v[0], v[1], v[2], v[3]);
    return Vector{result.x, result.y, result.z, result.w};
  }
  else return v;
}

/**
 * @brief Returns the unit normal vector for a plane containing the 3 given
 *        points which are expected to have a counterclockwise winding order.
 */
Vector planeNormal(const Vector& p1, const Vector& p2, const Vector& p3) {
  return normalize(vecCross(p2-p1, p3-p2));
}


VecList operator+(VecList l1, VecList l2) {
  int size = l1.size();
  VecList sum(size);
  for(int i=0; i<size; i++)
    sum[i] = l1[i] + l2[i];
  return sum;
}

VecList operator+(double scalar, VecList list) {
  int size = list.size();
  VecList sum(size);
  for(int i=0; i<size; i++) {
    sum[i] = scalar + list[i];
  }
  return sum;
}

VecList operator+(VecList list, double scalar) {
  return scalar + list;
}

VecList operator*(VecList l1, VecList l2) {
  int size = l1.size();
  VecList product(size);
  for(int i=0; i<size; i++)
    product[i] = l1[i] * l2[i];
  return product;
}

VecList operator*(double scalar, VecList list) {
  int size = list.size();
  VecList product(size);
  for(int i=0; i<size; i++)
    product[i] = scalar * list[i];
  return product;
}

VecList operator*(VecList list, double scalar) {
  return scalar * list;
}
