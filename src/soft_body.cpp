#include "soft_body.h"
#include "rigid_body.h"


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
  boundingRadius = softBody.boundingRadius;

  surfaceMasses = std::vector<Mass*>(surfaceMassIndices.size());
  solver = softBody.solver;
  initSurfaceMasses();
  initSolver();
}


/**
 * @brief  Soft body constructor.
 * 
 * @param  masses              Point masses in the mass-spring structure.
 * @param  springs             Springs holding the masses together.
 * @param  surfaceMassIndices  Indices of the masses on the surface of the body.
 * @param  boundingRadius      Radius of the bounding sphere for collision.
 * @param  mass                Mass of the soft body.
 * @param  massRadii           Radius of point masses; for internal collision.
 * @param  friction            Damping coefficient for extra stabilizing force.
 */
SoftBody::SoftBody(const std::vector<Mass>& masses, const std::vector<Spring>& springs,
                   const std::vector<int>& surfaceMassIndices, double boundingRadius, double mass,
                   double massRadii, double friction)
{
  this->masses = masses;
  this->springs = springs;
  this->surfaceMassIndices = surfaceMassIndices;
  this->boundingRadius = boundingRadius;
  this->mass = mass;
  this->massRadii = massRadii;
  this->friction = friction;
  this->surfaceMasses = std::vector<Mass*>(surfaceMassIndices.size());
  initSurfaceMasses();

  VecList state(masses.size());
  getState(state);
  initSolver(state);
}


void SoftBody::initSolver(const VecList& state) {
  if(state.size() != 0)
    solver.setState(state);

  using namespace std::placeholders;
  solver.setODEfunction(std::bind(&SoftBody::ode, this, _1, _2, _3));
}

void SoftBody::initSurfaceMasses() {
  for(int i=0; i<surfaceMasses.size(); i++)
    surfaceMasses[i] = &masses[surfaceMassIndices[i]];
}

/**
 * @brief  Copies the state from each mass.
 * @param  state  Destination VecList.
 */
void SoftBody::getState(VecList& state) const {
  for(int i=0; i<masses.size(); i++)
    state[i] = masses[i].getState();
}

const std::vector<Mass*>& SoftBody::getSurfaceMasses() const {
  return surfaceMasses;
}

double SoftBody::getBoundingRadius() const {
  return boundingRadius;
}


/**
 * @brief Set of ordinary differential equations that control the state of the
 *        soft body. Meant to be used by a MultiStateRK4solver object.
 *
 * @param  rates   Destination VecList that's populated with a dState/dt vector
 *                 for each state vector in the given list.
 * @param  states  List of states calculated from the previous iteration.
 * @param  time    Time which the ODEs are integrated up to.
 */
void SoftBody::ode(VecList& rates, const VecList& states, double time) const {
  for(int i=0; i<states.size(); i++)                               // Change in position
    rates[i][Mass::POS] = states[i][Mass::VEL];

  for(const Spring& spring : springs) {                            // Change in velocity
    const std::pair<int, int>& sMasses = spring.getMassIndices();
    Vector force = spring.calculateForce(states[sMasses.first], states[sMasses.second], massRadii);

    // Spring force plus additional friction to stabilize structure
    rates[sMasses.first][Mass::VEL]  += force - friction*states[sMasses.first][Mass::VEL];
    rates[sMasses.second][Mass::VEL] -= force + friction*states[sMasses.second][Mass::VEL];
  }
}


void SoftBody::handleCollision(double tStart, double tColl, const RigidRectPrism* rigidBody,
                               int massIndex, int faceIndex)
{
  double tEnd = solver.getTime();
  VecList stateStart(masses.size());
  getState(stateStart);
  solver.setState(stateStart, tStart);

  const Quad&   face       = rigidBody->getFaces()[faceIndex];
  const Vector& faceNormal = face.normal;
  const Vector& facePoint  = rigidBody->getVertices()[face.vertices[0]];

  const VecList& state     = solver.integrate(tColl);
  const Vector&  direction = state[massIndex][Mass::POS] - facePoint;
  double         distance  = vecDot(direction, faceNormal);
}


/**
 * @brief Calculates the updated state of the soft body at the given time using
 *        the given number of RK4iterations.
 */
const VecList& SoftBody::calculateUpdatedState(double time, int RK4iterations) {
  return solver.integrate(time, RK4iterations);
}

/**
 * @brief  Sets the state of each mass in the soft body.
 * @param  states  Updated states.
 */
void SoftBody::update(const VecList& states) {
  for(int i=0; i<masses.size(); i++)
    masses[i].update(states[i]);
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
  return state[POS];
}

Vector Mass::getVel() const {
  return state[VEL];
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
 * @brief  Calculates the force exerted on the masses on either end of the
 *         spring.
 *
 * @param  m1State    State of the first mass attached to the spring.
 * @param  m2State    State of the second mass attached to the spring.
 * @param  massRadii  Radius of the masses for collision detection.
 *
 * @return The force exerted on the masses.
 */
Vector Spring::calculateForce(const Vector& m1State, const Vector& m2State, double massRadii) const {
  Vector force(3);

  // Relative velocity and direction
  Vector velocity  = m1State[Mass::VEL] - m2State[Mass::VEL];
  Vector direction = m1State[Mass::POS] - m2State[Mass::POS];

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
