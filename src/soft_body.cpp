#include "soft_body.h"
#include "rigid_body.h"


SoftBody::SoftBody(double time) {
  this->time = time;
}

SoftBody::SoftBody(const SoftBody& softBody) {
  this->masses         = softBody.masses;
  this->springs        = softBody.springs;
  this->surfaceMasses  = softBody.surfaceMasses;
  this->boundingRadius = softBody.boundingRadius;
  this->time           = softBody.time;
  this->solver         = softBody.solver;
  this->restCollisions = softBody.restCollisions;
  initSolver();
}


/**
 * @brief  Soft body constructor.
 * 
 * @param  masses          Point masses in the mass-spring structure.
 * @param  springs         Springs holding the masses together.
 * @param  surfaceMasses   Indices of the masses on the surface of the body.
 * @param  boundingRadius  Radius of the bounding sphere for collision.
 * @param  time            Time of current state.
 */
SoftBody::SoftBody(const std::vector<Mass>& masses, const std::vector<Spring>& springs,
                   const std::vector<int>& surfaceMasses, double boundingRadius, double time)
{
  this->masses         = masses;
  this->springs        = springs;
  this->surfaceMasses  = surfaceMasses;
  this->boundingRadius = boundingRadius;
  this->time           = time;

  VecList state(masses.size());
  getState(state);
  initSolver(state);
}


/**
 * @brief  Binds the ODE function to the RK4 solver and sets the state if given.
 * @param  state  Initial state for RK4 solver.
 */
void SoftBody::initSolver(const VecList& state) {
  if(state.size() != 0)
    solver.setState(state);

  using namespace std::placeholders;
  solver.setODEfunction(std::bind(&SoftBody::ode, this, _1, _2, _3));
}

/**
 * @brief  Copies the state from each mass.
 * @param  state  Destination VecList.
 */
void SoftBody::getState(VecList& state) const {
  for(int i=0; i<masses.size(); i++)
    state[i] = masses[i].getState();
}

const std::vector<Mass>& SoftBody::getMasses() const            { return masses; }
const std::vector<int>& SoftBody::getSurfaceMassIndices() const { return surfaceMasses; }
double SoftBody::getBoundingRadius() const                      { return boundingRadius; }


/**
 * @brief Set of ordinary differential equations that control the state of the
 *        soft body. Meant to be used by a MultiStateRK4solver object.
 *
 * @param  rate   Destination VecList that's populated with a dState/dt vector
 *                for each state vector in the given list.
 * @param  state  List of states calculated from the previous iteration.
 * @param  time   Time which the ODEs are integrated up to.
 */
void SoftBody::ode(VecList& rate, const VecList& state, double time) {
  for(int i=0; i<state.size(); i++) {
    rate[i][Mass::POS] = state[i][Mass::VEL];   // Change in position
    rate[i][5]         = -9.8;                  // Gravity
  }

  // Spring forces
  for(Spring& spring : springs) {
    std::pair<int, int> sMasses = spring.getMassIndices();
    Vector force = spring.calculateForce(state[sMasses.first], state[sMasses.second]);
    rate[sMasses.first][Mass::VEL]  += force;
    rate[sMasses.second][Mass::VEL] -= force;
  }

  // Resting contact
  for(const auto& coll : restCollisions) {
    Vector&        massRate   = rate[coll.first];
    const Surface& surface    = coll.second;                 // Surface the mass is resting on
    const Vector&  normalV    = *surface.normal;             // Normal vector of surface
    Vector         dPos       = massRate[Mass::POS];         // Change in position
    Vector         force      = massRate[Mass::VEL];         // Force on mass
    double         normalF    = vecDot(force, -normalV);     // Normal force
    double         dPosNormal = vecDot(dPos, normalV);       // Position change parallel to normal
    Vector         dPosOrthog = dPos - dPosNormal*normalV;   // Position change orthogonal to normal

    // Normal force
    if(normalF > 0)
      massRate[Mass::VEL] += normalF * normalV;

    // Static friction
    if(vecNorm(dPosOrthog) < 1e-8) {
      Vector forceOrthog = force + normalF*normalV;
      if(vecNorm(forceOrthog) <= surface.staticFriction * normalF) {
        massRate[Mass::VEL] -= forceOrthog;
      }
    }

    // Kinematic friction
    else massRate[Mass::VEL] -= surface.kineticFriction * normalF * normalize(dPosOrthog);
  }
}


/**
 * @brief  Advances soft body to linearly approximated collision time and
 *         calculates the state of the mass after colliding with the surface.
 *         RK4 solver state buffer should be at or prior to the collision state,
 *         and will be at the time of the collision when the function returns.
 *
 * @param  tColl      Approximate collision time.
 * @param  collPoint  Approximate collision point.
 * @param  surface    Surface the mass is colliding with.
 * @param  massIndex  Index of mass involved in collision.
 * @param  e          Elasticity of collision.
 */
void SoftBody::handleCollision(double tColl, Vector& collPoint, const Surface& surface,
                               int massIndex, double e)
{
  solver.integrate(tColl);

  const Vector& massState = solver.getState()[massIndex];
  const Vector& normal    = *surface.normal;
  double        velNormal = vecDot(massState[Mass::VEL], normal);

  // Calculate post-collision state
  Vector massStateColl(massState);
  massStateColl[Mass::POS] = collPoint;
  if(velNormal < 0)
    massStateColl[Mass::VEL] -= (1+e) * velNormal * normal;

  solver.setSingleState(massIndex, massStateColl);
  masses[massIndex].update(massStateColl);

  // Initiate resting contact
  restCollisions.push_back(std::make_pair(massIndex, surface));
}


/**
 * @brief Checks if masses have moved away from the surface they were resting on
 *        and removes their collision from the list if so.
 */
void SoftBody::updateRestCollisions() {
  restCollisions.remove_if([this](const std::pair<int, Surface>& coll) {
    const VecList& state = solver.getState();
    const Vector&  dPos  = Vector(state[coll.first][Mass::POS]) - masses[coll.first].getPos();
    return vecDot(dPos, *coll.second.normal) > 0;
  });
}

/**
 * @brief Resets the RK4 solver state buffer to the state currently stored in
 *        the masses along with the corresponding time of that state. Meant to
 *        revert back to previous state for collision detection.
 */
void SoftBody::resetStateBuffer() {
  if(solver.getTime() == this->time)
    return;

  VecList state(masses.size());
  getState(state);
  solver.setState(state, this->time);
}

/**
 * @brief Calculates the updated state of the soft body at the given time using
 *        the given number of RK4iterations.
 */
const VecList& SoftBody::calculateUpdatedState(double time, int RK4iterations) {
  return solver.integrate(time, RK4iterations);
}

/**
 * @brief Updates the soft body to match the state from the RK4 solver buffer.
 */
void SoftBody::update() {
  updateRestCollisions();
  time = solver.getTime();

  const VecList& state = solver.getState();
  for(int i=0; i<masses.size(); i++)
    masses[i].update(state[i]);
}




/*******************************************************************************
 *  MASS
 ******************************************************************************/

std::slice const Mass::POS(0, 3, 1);
std::slice const Mass::VEL(3, 3, 1);

Mass::Mass(Vector pos, Vector vel) {
  this->state      = Vector(6);
  this->state[POS] = pos;
  this->state[VEL] = vel;
}

const Vector& Mass::getState() const   { return state;        }
Vector Mass::getPos() const            { return state[POS];   }
Vector Mass::getVel() const            { return state[VEL];   }
void Mass::update(const Vector& state) { this->state = state; }




/*******************************************************************************
 *  SPRING
 ******************************************************************************/

/**
 * @brief  Spring constructor.
 *
 * @param  mass1    First mass connected to spring.
 * @param  mass2    Second mass connected to spring.
 * @param  k        Spring coefficient.
 * @param  c        Damping coefficient.
 * @param  restLen  Rest length of the spring.
 */
Spring::Spring(int mass1, int mass2, double k, double c, double restLen) {
  this->masses  = std::make_pair(mass1, mass2);
  this->k       = k;
  this->c       = c;
  this->restLen = restLen;
}

const std::pair<int, int>& Spring::getMassIndices() const {
  return masses;
}


/**
 * @brief  Calculates the force exerted on the masses connected to either end of
 *         the spring.
 *
 * @param  m1State    State of the first mass attached to the spring.
 * @param  m2State    State of the second mass attached to the spring.
 *
 * @return Force vector.
 */
Vector Spring::calculateForce(const Vector& m1State, const Vector& m2State) {
  Vector velocity    = m1State[Mass::VEL] - m2State[Mass::VEL];
  Vector direction   = m1State[Mass::POS] - m2State[Mass::POS];
  double length      = vecNorm(direction);
  Vector d           = direction / (length>0.0? length : 1e-8);
  double deformation = length - restLen;

  return -k*deformation*d - c*velocity;
}
