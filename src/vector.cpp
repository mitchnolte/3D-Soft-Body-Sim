#include "vector.h"

float vecDot(const Vector& u, const Vector& v) {return (u * v).sum();}
float vecNorm(const Vector& v)                 {return sqrt(vecDot(v, v));}

VecList operator+(VecList l1, VecList l2) {
  int size = l1.size();
  VecList sum(size);
  for(int i=0; i<size; i++)
    sum[i] = l1[i] + l2[i];
  return sum;
}

VecList operator*(VecList l1, VecList l2) {
  int size = l1.size();
  VecList product(size);
  for(int i=0; i<size; i++)
    product[i] = l1[i] * l2[i];
  return product;
}

VecList operator*(float scalar, VecList list) {
  int size = list.size();
  VecList product(size);
  for(int i=0; i<size; i++)
    product[i] = scalar * list[i];
  return product;
}

VecList operator*(VecList list, float scalar) {
  return scalar * list;
}
