#include "soft_body.h"
#include "rigid_body.h"


/*******************************************************************************
 *  SOFT BODY
 ******************************************************************************/

SoftBody::SoftBody(double time) {
  this->time = time;
}

SoftBody::SoftBody(const SoftBody& softBody) {
  this->centerOfMass   = softBody.centerOfMass;
  this->masses         = softBody.masses;
  this->springs        = softBody.springs;
  this->surfaceMasses  = softBody.surfaceMasses;
  this->mass           = softBody.mass;
  this->massRadii      = softBody.massRadii;
  this->boundingRadius = softBody.boundingRadius;
  this->time           = softBody.time;

  this->solver = softBody.solver;
  initSolver();
}


/**
 * @brief  Soft body constructor.
 * 
 * @param  masses          Point masses in the mass-spring structure.
 * @param  springs         Springs holding the masses together.
 * @param  surfaceMasses   Indices of the masses on the surface of the body.
 * @param  boundingRadius  Radius of the bounding sphere for collision.
 * @param  mass            Mass of the soft body.
 * @param  massRadii       Radius of point masses; for internal collision.
 * @param  time            Time of current state.
 */
SoftBody::SoftBody(const std::vector<Mass>& masses, const std::vector<Spring>& springs,
                   const std::vector<int>& surfaceMasses, double boundingRadius, double mass,
                   double massRadii, double time)
{
  this->masses         = masses;
  this->springs        = springs;
  this->surfaceMasses  = surfaceMasses;
  this->boundingRadius = boundingRadius;
  this->mass           = mass;
  this->massRadii      = massRadii;
  this->time           = time;

  VecList state(masses.size());
  getState(state);
  initSolver(state);
  initSprings();
}


void SoftBody::initSolver(const VecList& state) {
  if(state.size() != 0)
    solver.setState(state);

  using namespace std::placeholders;
  solver.setODEfunction(std::bind(&SoftBody::ode, this, _1, _2, _3));
}

void SoftBody::initSprings() {
  for(Spring& spring : springs) {
    std::pair<int, int> indices = spring.getMassIndices();
    spring.setDirection(masses[indices.first].getPos() - masses[indices.second].getPos());
  }
}

/**
 * @brief  Copies the state from each mass.
 * @param  state  Destination VecList.
 */
void SoftBody::getState(VecList& state) const {
  for(int i=0; i<masses.size(); i++)
    state[i] = masses[i].getState();
}

const std::vector<Mass>& SoftBody::getMasses() const {
  return masses;
}

const std::vector<int>& SoftBody::getSurfaceMassIndices() const {
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
void SoftBody::ode(VecList& rates, const VecList& states, double time) {
  for(int i=0; i<states.size(); i++) {
    rates[i][Mass::POS] = states[i][Mass::VEL];   // Change in position
    rates[i][5]         = -9.8;                   // Gravity
  }

  // Spring forces
  for(Spring& spring : springs) {
    std::pair<int, int> sMasses = spring.getMassIndices();
    const Vector& force = spring.calculateForce(states[sMasses.first], states[sMasses.second], massRadii);
    rates[sMasses.first]  += force;
    rates[sMasses.second] -= force;
  }
}













#include <iostream>


















/**
 * @brief  Finds the exact time of collision (up to a given amount of precision)
 *         between a mass and a rigid body face based on the linearly
 *         approximated collision time and calculates the new mass state. When
 *         function returns, buffer state will be at time of collision with the
 *         post-collision state of the mass involved in the collision.
 *
 * @param  tColl               Approximate collision time.
 * @param  tEnd                End time of the update step.
 * @param  rigidBody           Rigid body involved in collision.
 * @param  massIndex           Index of mass involved in collision.
 * @param  faceIndex           Index of face mass collided with.
 * @param  e                   Elasticity coefficient.
 * @param  collisionTolerance  Error tolerance for collision distance.
 *
 * @return Time of collision.
 */
double SoftBody::handleCollision(double tColl, double tEnd, const RigidRectPrism* rigidBody,
                                 int massIndex, int faceIndex, double e, double collisionTolerance)
{

  // printf("Handling collision: ");


  const VecList* state     = &solver.getState();
  const Vector&  massState = (*state)[massIndex];
  const Quad&    face      = rigidBody->getFaces()[faceIndex];
  const Vector&  facePoint = rigidBody->getVertices()[face.vertices[0]];

  double tStart = solver.getTime();
  double t      = tColl;
  double distance;                    // Signed distance from mass to face plane


  // std::cout << tStart << ", " << tColl << ", " << tEnd << "\n";


//   if(t <= tStart) {
    distance = vecDot(massState[Mass::POS] - facePoint, face.normal);
//     
//   }
// 
//   else {
//     VecList stateStart(masses.size());
//     getState(stateStart);
// 
//     // Integrate up to approximated collision time
//     state    = &solver.integrate(t, restCollisions);
//     distance = vecDot(massState[Mass::POS] - facePoint, face.normal);
// 
// 
//     printf("Searching for collision time in (");
// 
// 
//     if(distance > 0) {
//       stateStart = *state;
//       tStart = tColl;
//     }else
//       tEnd = tColl;
// 
// 
//     std::cout << tStart << ", " << tEnd << "): ";
// 
// 
//     // Binary search for collision time
//     while(fabs(distance) > collisionTolerance) {
// 
// 
//       printf("|");
// 
// 
//       t = 0.5*(tStart + tEnd);
//       solver.setState(stateStart, tStart);
//       state     = &solver.integrate(t, restCollisions);
//       distance  = vecDot(massState[Mass::POS] - facePoint, face.normal);
//       if(distance > 0) {
//         stateStart = *state;
//         tStart = t;
//       }else
//         tEnd = t;
//     }
// 
// 
//     printf("\n");
//   }


  // Calculate state after collision
  Vector massStateColl(massState);
  massStateColl[Mass::VEL] -= (1+e) * face.normal * vecDot(massState[Mass::VEL], face.normal);
  massStateColl[Mass::POS] += distance * face.normal;


  // std::cout<<"t = "<<t<<" | d = "<<distance<<" | ct = "<<collisionTolerance<<" | Pos: ("<<massState[0]<<", "<<massState[1]<<", "<<massState[2]<<") --> ("<<massStateColl[0]<<", "<<massStateColl[1]<<", "<<massStateColl[1]<<")\t";
  // std::cout<<"| Vel: ("<<massState[3]<<", "<<massState[4]<<", "<<massState[5]<<") --> ("<<massStateColl[3]<<", "<<massStateColl[4]<<", "<<massStateColl[5]<<")\n";


  solver.setSingleState(massIndex, massStateColl);

  const std::pair<double, double>& frictionCoeffs = rigidBody->getFrictionCoefficients();
  restCollisions[massIndex] = {&face.normal, frictionCoeffs.first, frictionCoeffs.second};
  return t;
}


/**
 * @brief Calculates the updated state of the soft body at the given time using
 *        the given number of RK4iterations.
 */
const VecList& SoftBody::calculateUpdatedState(double time, int RK4iterations) {
  return solver.integrate(time, restCollisions, RK4iterations);
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
 * @brief Flushes the RK4 solver state buffer to the masses.
 */
void SoftBody::flushStateBuffer() {
  time = solver.getTime();
  const VecList& state = solver.getState();
  for(int i=0; i<masses.size(); i++)
    masses[i].update(state[i]);
  approximateCOM(state);
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
  this->masses  = std::make_pair(mass1, mass2);
  this->k       = k;
  this->c       = c;
  this->restLen = restLen;
}

void Spring::setDirection(const Vector& direction) {
  this->direction = direction;
}

const std::pair<int, int>& Spring::getMassIndices() const {
  return masses;
}


/**
 * @brief  Calculates the force exerted on the masses on either end of the
 *         spring and checks if they have collided.
 *
 * @param  m1State    State of the first mass attached to the spring.
 * @param  m2State    State of the second mass attached to the spring.
 * @param  massRadii  Radius of the masses for collision detection.
 *
 * @return Rate vector for spring force and collision.
 */
Vector Spring::calculateForce(const Vector& m1State, const Vector& m2State, double massRadii) {
  Vector dState(6);

  // Relative velocity and direction
  Vector velocity  = m1State[Mass::VEL] - m2State[Mass::VEL];
  Vector direction = m1State[Mass::POS] - m2State[Mass::POS];

  double length      = vecNorm(direction);              // Spring length
  Vector u           = direction / length;              // Unit length direction
  double deformation = length - restLen;                // Spring deformation
  dState[Mass::VEL]  = -k*deformation*u - c*velocity;   // Spring force

  // Internal mass collision
  // if(length <= 2*massRadii) {
  //   double velU = vecDot(velocity, u);
  //   if(velU > 0) {
  //     dState[Mass::POS] += 2.0 * u * velU;
  //     // printf("%f, %f, %f\n", dState[0], dState[1], dState[2]);
  //   }
  //   // force += 0.2*u / (length * length);
  // }
  // double distance = vecDot(direction, this->direction);
  // if(distance < 0) {
  //   // dState[Mass::POS] += 2.0 * u * vecDot(velocity, u);
  //   dState[Mass::POS] += 2.0 * this->direction * vecDot(velocity, this->direction);
  //   printf("%f, %f, %f\n", dState[0], dState[1], dState[2]);
  // }

  // double distance = vecDot(direction, this->direction);
  // if(length <= 2*massRadii || distance < 0) {
  //   if(distance < 0)
  //     dState[Mass::POS] += 2.0 * u * vecDot(velocity, u);
  //   else
  //     dState[Mass::POS] -= 2.0 * u * vecDot(velocity, u);
  // }
  // this->direction = u;

  return dState;
}
