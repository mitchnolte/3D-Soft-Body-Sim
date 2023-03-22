#include "soft_body.h"


/*******************************************************************************
 *  SOFT BODY
 ******************************************************************************/

SoftBody::SoftBody() {
  using namespace std::placeholders;
  solver.setODEfunction(std::bind(&SoftBody::ode, this, _1, _2));
}

SoftBody::SoftBody(const SoftBody& softBody) {
  masses = softBody.masses;
  springs = softBody.springs;
  surfaceMassIndices = softBody.surfaceMassIndices;
  mass = softBody.mass;
  solver = softBody.solver;

  surfaceMasses = std::vector<Mass*>(surfaceMassIndices.size());
  for(int i=0; i<surfaceMasses.size(); i++) {
    surfaceMasses[i] = &this->masses[surfaceMassIndices[i]];
  }
}

SoftBody::SoftBody(const std::vector<Mass>& masses, const std::vector<Spring>& springs,
                   const std::vector<int>& surfaceMassIndices)
{
  this->masses = masses;
  this->springs = springs;
  this->surfaceMassIndices = surfaceMassIndices;
  this->surfaceMasses = std::vector<Mass*>(surfaceMassIndices.size());
  for(int i=0; i<surfaceMasses.size(); i++) {
    surfaceMasses[i] = &this->masses[surfaceMassIndices[i]];
  }

  VecList state(masses.size());
  for(int i=0; i<masses.size(); i++) {
    state[i] = masses[i].getState();
  }

  using namespace std::placeholders;
  solver.setODEfunction(std::bind(&SoftBody::ode, this, _1, _2));
  solver.setState(state);
}

const std::vector<Mass*>& SoftBody::getSurfaceMasses() const {
  return surfaceMasses;
}


void SoftBody::update(double time) {
  VecList states = solver.integrate(time);

  // TODO: check for internal collisions

  for(int i=0; i<masses.size(); i++) {
    masses[i].update(states[i]);
  }
}


VecList SoftBody::ode(const VecList& states, double time) {
  VecList rates = VecList(states.size(), Vector(states[0].size()));
  for(Spring spring : springs) {
    int* springMasses = spring.getMassIndices();
    Vector force = spring.calculateForce(states[springMasses[0]], states[springMasses[1]]);
    rates[springMasses[0]] += force;
    rates[springMasses[1]] -= force;
  }

  return rates;
}


/*******************************************************************************
 *  MASS
 ******************************************************************************/

Mass::Mass(Vector pos, Vector vel) {
  this->state = Vector(6);
  this->state[std::slice(0, 3, 1)] = pos;
  this->state[std::slice(3, 3, 1)] = vel;
}

const Vector& Mass::getState() const {
  return state;
}

Vector Mass::getPos() const {
  return Vector(state[std::slice(0, 3, 1)]);
}

Vector Mass::getVel() const {
  return Vector(state[std::slice(3, 3, 1)]);
}

void Mass::update(const Vector& state) {
  this->state = state;
}


/*******************************************************************************
 *  SPRING
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


/**
 * @brief Calculates the force exerted on the masses on either end of the
 *        spring.
 * @param m1State State of the first mass attached to the spring.
 * @param m2State State of the second mass attached to the spring.
 * @return The force exerted on the masses.
 */
Vector Spring::calculateForce(const Vector& m1State, const Vector& m2State) {

  // Relative velocity and direction
  Vector velocity  = Vector(m1State[std::slice(3, 3, 1)]) - Vector(m2State[std::slice(3, 3, 1)]);
  Vector direction = Vector(m1State[std::slice(0, 3, 1)]) - Vector(m2State[std::slice(0, 3, 1)]);

  float  length      = vecNorm(direction);              // Spring length
  Vector u           = direction / length;              // Unit length direction
  float  deformation = length - restLen;                // Spring deformation
  Vector force       = -k*deformation*u - c*velocity;

  return force;
}
