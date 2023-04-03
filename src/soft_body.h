#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include <GL/glew.h>
#include "vector.h"
#include "rk4_solver.h"
class RigidRectPrism;

class Mass;
class Spring;


/**
 * @brief Interface for a soft body. Implementations define the shape of the
 *        mass-spring structure.
 */
class SoftBody {
protected:
  std::vector<Mass> masses;
  std::vector<Spring> springs;
  std::vector<Mass*> surfaceMasses;     // List of pointers to surface masses
  std::vector<int> surfaceMassIndices;  // Indices of surface masses in mass list
  double mass;                          // Overall body mass
  double massRadii;                     // Radius of a single point mass for internal collision
  MultiStateRK4solver solver;           // ODE solver
  double friction;                      // Friction coefficient
  double boundingRadius;                // Radius of bounding sphere for collision detection


  void initSolver(const VecList& state=VecList());
  void initSurfaceMasses();

public:
  SoftBody();
  SoftBody(const SoftBody& softBody);
  SoftBody(const std::vector<Mass>& masses, const std::vector<Spring>& springs,
           const std::vector<int>& surfaceMassIndices, double boundingRadius,
           double mass, double massRadii, double friction);
  void getState(VecList& state) const;
  const std::vector<Mass*>& getSurfaceMasses() const;
  double getBoundingRadius() const;
  void ode(VecList& rates, const VecList& states, double time) const;
  void handleCollision(double tStart, double tColl, const RigidRectPrism* rigidBody,
                       int massIndex, int faceIndex);

  const VecList& calculateUpdatedState(double time, int RK4iterations);
  void update(const VecList& states);

  virtual Vector getCenterOfMass() = 0;
};


/**
 * @brief Point mass used in the mass-spring structure of a soft body.
 */
class Mass {
  Vector state;    // Current state of mass; updated at end of time step

public:
  static const std::slice POS;  // Vector slice for the position of a mass state
  static const std::slice VEL;  // Vector slice for the velocity of a mass state

  Mass(Vector pos=Vector(3), Vector vel=Vector(3));
  const Vector& getState() const;
  Vector getPos() const;
  Vector getVel() const;
  void update(const Vector& state);
};


/**
 * @brief Spring used in the mass-spring structure of a soft body. Responsible
 *        for calculating the force on the point masses connected to either end.
 */
class Spring {
  std::pair<int, int> masses; // Indices of connected masses in soft body mass list
  double k;                   // Spring coefficient
  double c;                   // Damping coefficient
  double restLen;             // Length of spring at rest

public:
  Spring(int mass1, int mass2, double k, double c, double restLen);
  const std::pair<int, int>& getMassIndices() const;
  Vector calculateForce(const Vector& m1State, const Vector& m2State, double massRadii) const;
};

#endif
