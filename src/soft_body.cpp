#include "soft_body.h"


/*******************************************************************************
 *  SOFT BODY
 ******************************************************************************/

SoftBody::SoftBody() {}

SoftBody::SoftBody(const SoftBody& softBody) {
  masses = softBody.masses;
  springs = softBody.springs;
  surfaceMassIndices = softBody.surfaceMassIndices;
  mass = softBody.mass;
  massRadii = softBody.massRadii;
  friction = softBody.friction;

  surfaceMasses = std::vector<Mass*>(surfaceMassIndices.size());
  for(int i=0; i<surfaceMasses.size(); i++) {
    surfaceMasses[i] = &this->masses[surfaceMassIndices[i]];
  }
  
  using namespace std::placeholders;
  solver = softBody.solver;
  solver.setODEfunction(std::bind(&SoftBody::ode, this, _1, _2, _3));
}

SoftBody::SoftBody(const std::vector<Mass>& masses, const std::vector<Spring>& springs,
         const std::vector<int>& surfaceMassIndices, double mass, double massRadii, double friction)
{
  this->masses = masses;
  this->springs = springs;
  this->surfaceMassIndices = surfaceMassIndices;
  this->mass = mass;
  this->massRadii = massRadii;
  this->friction = friction;
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


void SoftBody::update(double time, int RK4iterations) {
  VecList states = solver.integrate(time, RK4iterations);

  for(int i=0; i<masses.size(); i++) {
    masses[i].update(states[i]);
  }
}


void SoftBody::ode(VecList& rates, const VecList& states, double time) const {
  for(int i=0; i<states.size(); i++)                               // Change in position
    rates[i][Mass::POS] = states[i][Mass::VEL];

  for(const Spring& spring : springs) {                            // Change in velocity
    const std::pair<int, int>& sMasses = spring.getMassIndices();
    Vector force = spring.calculateForce(states[sMasses.first], states[sMasses.second], massRadii);

    // Spring force + additional friction to stabilize structure
    rates[sMasses.first][Mass::VEL]  += force - friction*states[sMasses.first][Mass::VEL];
    rates[sMasses.second][Mass::VEL] -= force + friction*states[sMasses.second][Mass::VEL];
  }
}


/*******************************************************************************
 *  MASS
 ******************************************************************************/

std::slice const Mass::POS(0, 3, 1);
std::slice const Mass::VEL(3, 3, 1);

Mass::Mass(Vector pos, Vector vel) {
  this->state = Vector(6);
  this->state[POS] = pos;
  this->state[VEL] = vel;
}

const Vector& Mass::getState() const {
  return state;
}

Vector Mass::getPos() const {
  return Vector(state[POS]);
}

Vector Mass::getVel() const {
  return Vector(state[VEL]);
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
  Vector force(3);

  // Relative velocity and direction
  Vector velocity  = Vector(m1State[Mass::VEL]) - Vector(m2State[Mass::VEL]);
  Vector direction = Vector(m1State[Mass::POS]) - Vector(m2State[Mass::POS]);

  double length      = vecNorm(direction);      // Spring length
  Vector u           = direction / length;      // Unit length direction
  double deformation = length - restLen;        // Spring deformation
  if(fabs(deformation) > 0.0000000001)          // Account for precision error
    force = -k*deformation*u - c*velocity;      // Spring force

  // Internal mass collision
  // if(length <= 2*massRadii)
  //   force += u / (length * length);

  return force;
}
