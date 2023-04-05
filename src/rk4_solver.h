#ifndef RK4_SOLVER_H
#define RK4_SOLVER_H

#include <functional>
#include <unordered_map>
// #include "soft_body.h"
#include "vector.h"
struct Surface;

typedef std::function< void (Vector&,  const Vector&,  double) >  ODEfn;
typedef std::function< void (VecList&, const VecList&, double) >  MultiStateODEfn;


/**
 * @brief Solves a system of ordinary differential equations using the
 *        fourth-order Runge-Kutta method.
 */
class RK4solver {
  ODEfn f;
  double time;
  Vector state;

public:
  RK4solver();
  RK4solver(ODEfn odeFunction, const Vector& state=Vector(), double time=0.0);
  void setODEfunction(ODEfn odeFunction);
  void setState(const Vector& state, double time=-1.0);
  double getTime() const;
  const Vector& getState() const;
  const Vector& integrate(double time, int steps=1);
};


/**
 * @brief Solves a system of ordinary differential equations using the
 *        fourth-order Runge-Kutta method for a list of state vectors.
 */
class MultiStateRK4solver {
  MultiStateODEfn f;
  double time;
  VecList state;
  VecList k1;
  VecList k2;
  VecList k3;
  VecList k4;

  void handleRestCollisions(std::unordered_map<int, Surface>& restCollisions, VecList& newState);

public:
  MultiStateRK4solver();
  MultiStateRK4solver(MultiStateODEfn odeFunction, const VecList& state=VecList(), double time=0.0);
  void setODEfunction(MultiStateODEfn odeFunction);
  void setState(const VecList& state, double time=-1.0);
  void setSingleState(int index, const Vector& state);
  double getTime() const;
  const VecList& getState() const;
  const VecList& integrate(double time, std::unordered_map<int, Surface>& restCollisions,
                           int steps=1);
};

#endif
