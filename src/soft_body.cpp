#include "soft_body.h"

/*******************************************************************************
 *  SOFT BODY CLASS
 ******************************************************************************/

SoftBody::SoftBody() {}

void SoftBody::ode(const List<Vector>& states, List<Vector>& rates) {
  for(Spring spring : springs) {
    int* masses = spring.getMassIndices();
    Vector force = spring.calculateForces(states[masses[0]], states[masses[1]]);
    rates[masses[0]] += force;
    rates[masses[1]] -= force;
  }

  // TODO: check for internal collisions
}


/*******************************************************************************
 *  MASS CLASS
 ******************************************************************************/

Mass::Mass(Vector pos, Vector vel) {
  this->state = Vector(6);
  this->state[std::slice(0, 3, 1)] = pos;
  this->state[std::slice(3, 3, 1)] = vel;
}

Vector Mass::getPos() {
  return Vector(state[std::slice(0, 3, 1)]);
}

Vector Mass::getVel() {
  return Vector(state[std::slice(3, 3, 1)]);
}

void Mass::update(const Vector& state) {
  this->state = state;
}


/*******************************************************************************
 *  SPRING CLASS
 ******************************************************************************/

Spring::Spring(int mass1, int mass2, float k, float c, float restLen) {
  this->masses[0] = mass1;
  this->masses[1] = mass2;
  this->k = k;
  this->c = c;
  this->restLen = restLen;
}

int* Spring::getMassIndices() {
  return masses;
}

Vector Spring::calculateForces(const Vector& m1State, const Vector& m2State) {

  // Relative velocity between masses
  Vector velocity    = Vector(m1State[std::slice(3, 3, 1)]) - Vector(m2State[std::slice(3, 3, 1)]);

  // Direction from 2nd mass to 1st
  Vector direction   = Vector(m1State[std::slice(0, 3, 1)]) - Vector(m2State[std::slice(0, 3, 1)]);

  float  length      = vecNorm(direction);                      // Spring length
  Vector u           = direction / length;                      // Unit length direction
  float  deformation = length - restLen;                        // Spring deformation
  Vector force       = -k*deformation*u - c*velocity;

  return force;
}
