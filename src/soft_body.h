#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include <GL/glew.h>
#include "vector.h"
#include "rk4_solver.h"

class Mass;
class Spring;


class SoftBody {
  std::vector<Mass> masses;
  std::vector<Spring> springs;
  std::vector<Mass*> surfaceMasses;
  std::vector<int> surfaceMassIndices;
  float mass;
  MultiStateRK4solver solver;

public:
  SoftBody();
  SoftBody(const SoftBody& softBody);
  SoftBody(const std::vector<Mass>& masses, const std::vector<Spring>& springs,
           const std::vector<int>& surfaceMassIndices);
  const std::vector<Mass*>& getSurfaceMasses() const;
  void update(double time);
  VecList ode(const VecList& states, double time) const;
};


class Mass {
  Vector state;    // Current state of mass; updated at end of time step

public:
  Mass(Vector pos=Vector(3), Vector vel=Vector(3));
  const Vector& getState() const;
  Vector getPos() const;
  Vector getVel() const;
  void update(const Vector& state);
};


class Spring {
  std::pair<int, int> masses; // Indices of connected masses in soft body mass list
  float k;                    // Spring coefficient
  float c;                    // Damping coefficient
  float restLen;

public:
  Spring(int mass1, int mass2, float k, float c, float restLen);
  const std::pair<int, int>& getMassIndices() const;
  Vector calculateForce(const Vector& m1State, const Vector& m2State) const;
};

#endif
