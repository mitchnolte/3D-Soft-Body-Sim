#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include "vector.h"

class Mass;
class Spring;


class SoftBody {
  List<Mass> masses;
  List<Spring> springs;

public:
  SoftBody();

  /**
   *  **** TEMP COMMENT ****
   * 
   * - States separately stores the states for each mass (and the simulation
   *   will store these lists in another list with an entry for each object).
   * - Rates is just the output. It'll be quite large so I don't want to be
   *   copying it and don't feel like dealing with the memory management of
   *   dynamically allocating it.
   *     - Will need to initialize every Vector to (0, 0, 0) before calling this
   *       function (from sim class).
   */
  void ode(const List<Vector>& states, List<Vector>& rates);
};


class Mass {
  Vector state;    // Current state of mass; updated at end of time step

public:
  Mass(Vector pos=Vector(3), Vector vel=Vector(3));
  Vector getPos();
  Vector getVel();
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
