#include "rk4_solver.h"


RK4solver::RK4solver(ODEfn f, Vector state, double time) {
  this->f = f;
  this->state = state;
  this->time = time;
}

const Vector& RK4solver::getState() {
  return state;
}

void RK4solver::setState(const Vector& state, double time) {
  this->state = state;
}


const Vector& RK4solver::integrate(double time, int steps) {
  if(time <= this->time)
    return Vector();

  double t = this->time;
  float stepSize = (time - t) / steps;
  Vector k1, k2, k3, k4;
  
  for(int i=0; i<steps; i++) {
    k1 = stepSize * f(state, t);

    t += 0.5*stepSize;
    k2 = stepSize * f(state + 0.5f*k1, t);
    k3 = stepSize * f(state + 0.5f*k2, t);

    t += 0.5*stepSize;
    k4 = stepSize * f(state + k3, t);

    state += 0.16666667*(k1 + 2*(k2 + k3) + k4);
  }

  return state;
}



/*******************************************************************************
 *  MULTI-STATE SOLVER
 ******************************************************************************/

MultiStateRK4solver::MultiStateRK4solver(MultiStateODEfn f, VecList state, double time) {
  this->f = f;
  this->state = state;
  this->time = time;
}

const VecList& MultiStateRK4solver::getState() {
  return state;
}

void MultiStateRK4solver::setState(const VecList& state, double time) {
  this->state = state;
}


const VecList& MultiStateRK4solver::integrate(double time, int steps) {
  if(time <= this->time)
    return VecList();

  double t = this->time;
  float stepSize = (time - t) / steps;
  VecList k1, k2, k3, k4;
  
  for(int i=0; i<steps; i++) {
    k1 = stepSize * f(state, t);

    t += 0.5*stepSize;
    k2 = stepSize * f(state + 0.5f*k1, t);
    k3 = stepSize * f(state + 0.5f*k2, t);

    t += 0.5*stepSize;
    k4 = stepSize * f(state + k3, t);

    VecList dState = 0.16666667*(k1 + 2*(k2 + k3) + k4);
    for(int i=0; i<state.size(); i++) {
      state[i] += dState[i];
    }
  }

  return state;
}
