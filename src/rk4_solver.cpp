#include "rk4_solver.h"
#include "soft_body.h"


RK4solver::RK4solver() {
  time = 0;
}

RK4solver::RK4solver(ODEfn f, const Vector& state, double time) {
  this->f = f;
  this->state = state;
  this->time = time;
}

void RK4solver::setODEfunction(ODEfn odeFunction) {
  f = odeFunction;
}

void RK4solver::setState(const Vector& state, double time) {
  if(time >= 0.0) this->time = time;
  this->state = state;
}

double RK4solver::getTime() const {
  return time;
}

const Vector& RK4solver::getState() const {
  return state;
}


const Vector& RK4solver::integrate(double time, int steps) {
  if(time <= this->time)
    return state;

  double t = this->time;
  double stepSize = (time - t) / steps;
  double halfStep = 0.5 * stepSize;

  int stateSize = state.size();
  Vector k1(stateSize), k2(stateSize), k3(stateSize), k4(stateSize);

  for(int i=0; i<steps; i++) {
    k1 = 0;
    k2 = 0;
    k3 = 0;
    k4 = 0;

    f(k1, state, t);

    t += halfStep;
    f(k2, state + halfStep*k1, t);
    f(k3, state + halfStep*k2, t);

    t += halfStep;
    f(k4, state + stepSize*k3, t);

    state += stepSize * (1.0/6.0) * (k1 + 2*(k2 + k3) + k4);
  }

  this->time = t;
  return state;
}


/*******************************************************************************
 *  MULTI-STATE SOLVER
 ******************************************************************************/

MultiStateRK4solver::MultiStateRK4solver() {
  time = 0;
}

MultiStateRK4solver::MultiStateRK4solver(MultiStateODEfn f, const VecList& state, double time) {
  this->f = f;
  this->state = state;
  this->time = time;
  this->k1 = VecList(state.size(), Vector(state[0].size()));
  this->k2 = VecList(state.size(), Vector(state[0].size()));
  this->k3 = VecList(state.size(), Vector(state[0].size()));
  this->k4 = VecList(state.size(), Vector(state[0].size()));
}

void MultiStateRK4solver::setODEfunction(MultiStateODEfn odeFunction) {
  f = odeFunction;
}

void MultiStateRK4solver::setState(const VecList& state, double time) {
  if(time >= 0.0) this->time = time;
  this->state = state;

  if(k1.size() != state.size() || k1[0].size() != state[0].size()) {
    this->k1 = VecList(state.size(), Vector(state[0].size()));
    this->k2 = VecList(state.size(), Vector(state[0].size()));
    this->k3 = VecList(state.size(), Vector(state[0].size()));
    this->k4 = VecList(state.size(), Vector(state[0].size()));
  }
}

void MultiStateRK4solver::setSingleState(int index, const Vector& state) {
  this->state[index] = state;
}

double MultiStateRK4solver::getTime() const {
  return time;
}

const VecList& MultiStateRK4solver::getState() const {
  return state;
}

void MultiStateRK4solver::handleRestCollisions(std::unordered_map<int, Surface>& restCollisions,
                                               VecList& newState)
{
  for(const auto& coll : restCollisions) {
    Vector&        dState     = newState[coll.first];        // Change in state
    const Surface& surface    = coll.second;                 // Surface mass is in contact with
    const Vector&  normalV    = *surface.normal;             // Normal vector of surface
    Vector         dPos       = dState[Mass::POS];           // Change in position
    Vector         force      = dState[Mass::VEL];           // Force applied to mass
    double         normalF    = vecDot(force, -normalV);     // Normal force
    double         dPosNormal = vecDot(dPos, normalV);       // Position change parallel to normal
    Vector         dPosOrthog = dPos - dPosNormal*normalV;   // Position change orthogonal to normal

    // Stop mass from moving through surface
    if(dPosNormal < 0)
      dState[Mass::POS] -= dPosNormal*normalV;
    // else if(dPosN > 0)
    //   restCollisions.erase(coll.first);

    // Apply friction
    if(vecNorm(dPosOrthog) == 0) {
      double forceOrthog = vecNorm(force + normalF*normalV);
      if(forceOrthog <= surface.staticFriction * normalF)
        dState[Mass::VEL] = 0;
    }
    else dState[Mass::POS] -= surface.kineticFriction * normalF * normalize(dPosOrthog);
  }
}


const VecList& MultiStateRK4solver::integrate(double time,
                                        std::unordered_map<int, Surface>& restCollisions, int steps)
{
  if(time <= this->time)
    return state;

  double t = this->time;
  double stepSize = (time - t) / steps;
  double halfStep = 0.5 * stepSize;

  for(int i=0; i<steps; i++) {
    for(int j=0; j<state.size(); j++) {
      k1[j] = 0.0;
      k2[j] = 0.0;
      k3[j] = 0.0;
      k4[j] = 0.0;
    }

    f(k1, state, t);
    handleRestCollisions(restCollisions, k1);

    t += halfStep;
    f(k2, state + halfStep*k1, t);
    handleRestCollisions(restCollisions, k2);

    f(k3, state + halfStep*k2, t);
    handleRestCollisions(restCollisions, k3);

    t += halfStep;
    f(k4, state + stepSize*k3, t);
    handleRestCollisions(restCollisions, k4);

    const VecList& dState = stepSize * (1.0/6.0) * (k1 + 2*(k2 + k3) + k4);
    for(int j=0; j<state.size(); j++) {
      state[j] += dState[j];
    }
  }

  this->time = t;
  return state;
}
