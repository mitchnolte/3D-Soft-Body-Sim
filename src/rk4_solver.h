#ifndef RK4_SOLVER_H
#define RK4_SOLVER_H

#include <functional>
#include "vector.h"

typedef std::function< void (Vector&,  const Vector&,  double) >  ODEfn;
typedef std::function< void (VecList&, const VecList&, double) >  MultiStateODEfn;


class RK4solver {
  ODEfn f;
  double time;
  Vector state;

public:
  RK4solver();
  RK4solver(ODEfn odeFunction, const Vector& state=Vector(), double time=0.0);
  void setODEfunction(ODEfn odeFunction);
  void setState(const Vector& state, double time=0.0);
  const Vector& getState();
  const Vector& integrate(double time, int steps=4);
};


class MultiStateRK4solver {
  MultiStateODEfn f;
  double time;
  VecList state;
  VecList k1;
  VecList k2;
  VecList k3;
  VecList k4;

public:
  MultiStateRK4solver();
  MultiStateRK4solver(MultiStateODEfn odeFunction, const VecList& state=VecList(), double time=0.0);
  void setODEfunction(MultiStateODEfn odeFunction);
  void setState(const VecList& state, double time=0.0);
  const VecList& getState();
  const VecList& integrate(double time, int steps=1);
};


#endif
