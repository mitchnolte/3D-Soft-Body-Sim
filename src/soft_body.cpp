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

  surfaceMasses = std::vector<Mass*>(surfaceMassIndices.size());
  for(int i=0; i<surfaceMasses.size(); i++) {
    surfaceMasses[i] = &this->masses[surfaceMassIndices[i]];
  }
  
  using namespace std::placeholders;
  solver = softBody.solver;
  solver.setODEfunction(std::bind(&SoftBody::ode, this, _1, _2));
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


VecList SoftBody::ode(const VecList& states, double time) const {
  VecList rates = VecList(states.size(), Vector(states[0].size()));

  // Change in position
  for(int i=0; i<states.size(); i++) {
    rates[i][std::slice(0, 3, 1)] = states[i][std::slice(3, 3, 1)];
  }

  // Change in velocity
  for(const Spring& spring : springs) {
    const std::pair<int, int>& springMasses = spring.getMassIndices();
    Vector force = spring.calculateForce(states[springMasses.first], states[springMasses.second]);
    rates[springMasses.first][std::slice(3, 3, 1)]  += force;
    rates[springMasses.second][std::slice(3, 3, 1)] -= force;
    // printf("f = %f, %f, %f\n", force[0], force[1], force[2]);
  }

  
  // for(int i=0; i<rates.size(); i++) {
  //   printf("%f, %f, %f, %f, %f, %f\n", rates[i][0], rates[i][1], rates[i][2], rates[i][3], rates[i][4], rates[i][5]);
  // }
  // printf("\n");

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
Vector Spring::calculateForce(const Vector& m1State, const Vector& m2State) const {

  // Relative velocity and direction
  Vector velocity  = Vector(m1State[std::slice(3, 3, 1)]) - Vector(m2State[std::slice(3, 3, 1)]);
  Vector direction = Vector(m1State[std::slice(0, 3, 1)]) - Vector(m2State[std::slice(0, 3, 1)]);

  float  length      = vecNorm(direction);              // Spring length
  Vector u           = direction / length;              // Unit length direction
  float  deformation = length - restLen;                // Spring deformation
  Vector force       = -k*deformation*u - c*velocity;

  return force;
}
