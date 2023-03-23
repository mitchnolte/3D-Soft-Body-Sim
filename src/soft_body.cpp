#include "soft_body.h"


/*******************************************************************************
 *  SOFT BODY
 ******************************************************************************/

SoftBody::SoftBody(double mass, double massRadii) {
  this->mass = mass;
  this->massRadii = massRadii;

  using namespace std::placeholders;
  solver.setODEfunction(std::bind(&SoftBody::ode, this, _1, _2, _3));
}

SoftBody::SoftBody(const SoftBody& softBody) {
  masses = softBody.masses;
  springs = softBody.springs;
  surfaceMassIndices = softBody.surfaceMassIndices;
  mass = softBody.mass;
  massRadii = softBody.massRadii;

  surfaceMasses = std::vector<Mass*>(surfaceMassIndices.size());
  for(int i=0; i<surfaceMasses.size(); i++) {
    surfaceMasses[i] = &this->masses[surfaceMassIndices[i]];
  }
  
  using namespace std::placeholders;
  solver = softBody.solver;
  solver.setODEfunction(std::bind(&SoftBody::ode, this, _1, _2, _3));
}

SoftBody::SoftBody(const std::vector<Mass>& masses, const std::vector<Spring>& springs,
                   const std::vector<int>& surfaceMassIndices, double mass, double massRadii)
{
  this->masses = masses;
  this->springs = springs;
  this->surfaceMassIndices = surfaceMassIndices;
  this->mass = mass;
  this->massRadii = massRadii;
  this->surfaceMasses = std::vector<Mass*>(surfaceMassIndices.size());
  for(int i=0; i<surfaceMasses.size(); i++) {
    surfaceMasses[i] = &this->masses[surfaceMassIndices[i]];
  }

  VecList state(masses.size());
  for(int i=0; i<masses.size(); i++) {
    state[i] = masses[i].getState();
  }

  using namespace std::placeholders;
  solver.setODEfunction(std::bind(&SoftBody::ode, this, _1, _2, _3));
  solver.setState(state);
}

const std::vector<Mass*>& SoftBody::getSurfaceMasses() const {
  return surfaceMasses;
}


void SoftBody::update(double time) {
  VecList states = solver.integrate(time);

  for(int i=0; i<masses.size(); i++) {
    masses[i].update(states[i]);
  }
}


void SoftBody::ode(VecList& rates, const VecList& states, double time) const {

  // Change in position
  for(int i=0; i<states.size(); i++) {
    rates[i][std::slice(0, 3, 1)] = states[i][std::slice(3, 3, 1)];
  }

  // Change in velocity
  for(const Spring& spring : springs) {
    const std::pair<int, int>& springMasses = spring.getMassIndices();
    Vector force = spring.calculateForce(states[springMasses.first], states[springMasses.second],
                                         massRadii);

    // Additional friction to stabilize structure
    rates[springMasses.first][std::slice(3, 3, 1)]  += force - 0.008*states[springMasses.first][std::slice(3, 3, 1)];
    rates[springMasses.second][std::slice(3, 3, 1)] -= force + 0.008*states[springMasses.second][std::slice(3, 3, 1)];
  }
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

Spring::Spring(int mass1, int mass2, double k, double c, double restLen) {
  masses = std::make_pair(mass1, mass2);
  this->k = k;
  this->c = c;
  this->restLen = restLen;
}

const std::pair<int, int>& Spring::getMassIndices() const {
  return masses;
}


/**
 * @brief Calculates the force exerted on the masses on either end of the
 *        spring.
 * @param m1State State of the first mass attached to the spring.
 * @param m2State State of the second mass attached to the spring.
 * @return The force exerted on the masses.
 */
Vector Spring::calculateForce(const Vector& m1State, const Vector& m2State, double massRadii) const {

  // Relative velocity and direction
  Vector velocity  = Vector(m1State[std::slice(3, 3, 1)]) - Vector(m2State[std::slice(3, 3, 1)]);
  Vector direction = Vector(m1State[std::slice(0, 3, 1)]) - Vector(m2State[std::slice(0, 3, 1)]);

  double length      = vecNorm(direction);      // Spring length
  Vector u           = direction / length;      // Unit length direction
  double deformation = length - restLen;        // Spring deformation
  if(fabs(deformation) < 0.0000000001)          // Account for precision error
    return Vector(3);

  // Vector collisionForce(3);
  // if(length <= 2*massRadii)
  //   collisionForce = u / (length * length);

  // return -k*deformation*u - c*velocity + collisionForce;
  return -k*deformation*u - c*velocity;
}
