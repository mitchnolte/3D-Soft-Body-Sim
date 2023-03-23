#include "rk4_solver.h"


RK4solver::RK4solver() {}

RK4solver::RK4solver(ODEfn f, Vector state, double time) {
  this->f = f;
  this->state = state;
  this->time = time;
}

void RK4solver::setODEfunction(ODEfn odeFunction) {
  f = odeFunction;
}

void RK4solver::setState(const Vector& state, double time) {
  this->state = state;
}

const Vector& RK4solver::getState() {
  return state;
}


// const Vector& RK4solver::integrate(double time, int steps) {
//   if(time <= this->time)
//     return state;
// 
//   double t = this->time;
//   double stepSize = (time - t) / steps;
//   Vector k1, k2, k3, k4;
// 
//   for(int i=0; i<steps; i++) {
//     k1 = stepSize * f(state, t);
// 
//     t += 0.5*stepSize;
//     k2 = stepSize * f(state + 0.5*k1, t);
//     k3 = stepSize * f(state + 0.5*k2, t);
// 
//     t += 0.5*stepSize;
//     k4 = stepSize * f(state + k3, t);
// 
//     state += 0.16666667*(k1 + 2*(k2 + k3) + k4);
//   }
// 
//   return state;
// }


/*******************************************************************************
 *  MULTI-STATE SOLVER
 ******************************************************************************/

MultiStateRK4solver::MultiStateRK4solver() {
  time = 0;
  state = VecList();
}

MultiStateRK4solver::MultiStateRK4solver(MultiStateODEfn f, VecList state, double time) {
  this->f = f;
  this->state = state;
  this->time = time;
}

void MultiStateRK4solver::setState(const VecList& state, double time) {
  this->state = state;
}

void MultiStateRK4solver::setODEfunction(MultiStateODEfn odeFunction) {
  f = odeFunction;
}

const VecList& MultiStateRK4solver::getState() {
  return state;
}


const VecList& MultiStateRK4solver::integrate(double time, int steps) {
  if(time <= this->time)
    return state;

  double t = this->time;
  double stepSize = (time - t) / steps;
  double halfStep = 0.5 * stepSize;
  
  VecList k1(state.size(), Vector(state[0].size()));
  VecList k2(state.size(), Vector(state[0].size()));
  VecList k3(state.size(), Vector(state[0].size()));
  VecList k4(state.size(), Vector(state[0].size()));

  for(int i=0; i<steps; i++) {
//     // printf("\n\ni=%d, calculating k1:\n", i);
//     k1 = stepSize * f(state, t);
// 
//     // printf("\n\ni=%d, calculating k2:\n", i);
//     t += 0.5*stepSize;
//     k2 = stepSize * f(state + 0.5f*k1, t);
//   
//     // printf("\n\ni=%d, calculating k3:\n", i);
//     k3 = stepSize * f(state + 0.5f*k2, t);
// 
//     // printf("\n\ni=%d, calculating k4:\n", i);
//     t += 0.5*stepSize;
//     k4 = stepSize * f(state + k3, t);
// 
//     // VecList dState = 0.16666667*(k1 + 2*(k2 + k3) + k4);             // Original - square collapses quickly
//     // VecList dState = 0.166666667*(k1 + 2*(k2 + k3) + k4);            // Square collapses slower I think
//     // VecList dState = 0.1666666667*(k1 + 2*(k2 + k3) + k4);           // Nothing
//     // VecList dState = 0.16666666667*(k1 + 2*(k2 + k3) + k4);          // Nothing
//     // VecList dState = 0.166666666666666667*(k1 + 2*(k2 + k3) + k4);
//     // VecList dState = 1.0/6.0 *(k1 + 2*(k2 + k3) + k4);
// 
// 
//     for(int i=0; i<state.size(); i++) {
//       state[i] += dState[i];
//     }


    f(k1, state, t);
  
    t += halfStep;
    f(k2, state + halfStep*k1, t);
    f(k3, state + halfStep*k2, t);

    t += halfStep;
    f(k4, state + stepSize*k3, t);

    // VecList dState = 0.16666667 * stepSize * (k1 + 2*(k2 + k3) + k4);
    const VecList& dState = stepSize * (1.0/6.0) * (k1 + 2*(k2 + k3) + k4);
    for(int i=0; i<state.size(); i++) {
      state[i] += dState[i];
    }
  
    // Reset vector lists
    for(int i=0; i<state.size(); i++) {
      std::fill(std::begin(k1[i]), std::end(k1[i]), 0);
      std::fill(std::begin(k2[i]), std::end(k2[i]), 0);
      std::fill(std::begin(k3[i]), std::end(k3[i]), 0);
      std::fill(std::begin(k4[i]), std::end(k4[i]), 0);
    }
  }

  return state;
}
