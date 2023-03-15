#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include "vector.h"
#include "rk4_solver.h"

class Mass;
class Spring;


class SoftBody {
  std::vector<Mass> masses;
  std::vector<const Mass&> surfaceMasses;
  std::vector<Spring> springs;
  MultiStateRK4solver solver;

public:
  SoftBody();
  const std::vector<const Mass&>& getSurfaceMasses() const;
  void update(double time);
  VecList ode(const VecList& states, double time);
};


class Mass {
  Vector state;    // Current state of mass; updated at end of time step

public:
  Mass(Vector pos=Vector(3), Vector vel=Vector(3));
  Vector getPos() const;
  Vector getVel() const;
  void update(const Vector& state);
};


class Spring {
  int masses[2];
  float k;          // Spring coefficient
  float c;          // Damping coefficient
  float restLen;

public:
  Spring(int mass1, int mass2, float k, float c, float restLen);
  int* getMassIndices();
  Vector calculateForce(const Vector& m1State, const Vector& m2State);
};

#endif
