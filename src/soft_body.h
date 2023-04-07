#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include <GL/glew.h>
#include <list>
#include "vector.h"
#include "rk4_solver.h"
#include "collision_data.h"
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
  std::vector<int> surfaceMasses;   // Indices of surface masses in mass list
  double mass;                      // Overall body mass
  double massRadii;                 // Radius of a single point mass for internal collision
  double boundingRadius;            // Radius of bounding sphere for collision detection
  double time;                      // Time of state currently stored in masses
  MultiStateRK4solver solver;       // ODE solver

  // List of pairs storing the index of a mass and the properties of a surface
  // the mass is currently resting on.
  std::list<std::pair<int, Surface>> restCollisions;


  void initSolver(const VecList& state=VecList());

public:
  SoftBody(double time=0.0);
  SoftBody(const SoftBody& softBody);
  SoftBody(const std::vector<Mass>& masses, const std::vector<Spring>& springs,
           const std::vector<int>& surfaceMasses, double boundingRadius,
           double mass, double massRadii, double time=0.0);

  void getState(VecList& state) const;
  const std::vector<Mass>& getMasses() const;
  const std::vector<int>& getSurfaceMassIndices() const;
  double getBoundingRadius() const;
  void ode(VecList& rate, const VecList& state, double time);
  void handleCollision(double tColl, Vector& collPoint, const Surface& surface,
                       int massIndex, double e);

  const VecList& calculateUpdatedState(double time, int RK4iterations);
  void resetStateBuffer();
  void updateRestCollisions();
  void update();

  virtual Vector getCenterOfMass() const = 0;
};


/**
 * @brief Point mass used in the mass-spring structure of a soft body.
 */
class Mass {
  Vector state;     // Current state of mass; updated at end of time step
  bool colliding;   // Whether the mass is currently colliding with something

public:
  static const std::slice POS;  // Vector slice for the position of a mass state
  static const std::slice VEL;  // Vector slice for the velocity of a mass state

  Mass(Vector pos=Vector(3), Vector vel=Vector(3));
  const Vector& getState() const;
  Vector getPos() const;
  Vector getVel() const;
  bool isColliding();
  void isColliding(bool colliding);
  void update(const Vector& state);
};


/**
 * @brief Spring used in the mass-spring structure of a soft body. Responsible
 *        for calculating the force on the point masses connected to either end.
 */
class Spring {
  std::pair<int, int> masses; // Indices of connected masses in soft body mass list
  double restLen;             // Length of spring at rest
  double k;                   // Spring coefficient
  double c;                   // Damping coefficient

public:
  Spring(int mass1, int mass2, double k, double c, double restLen);
  const std::pair<int, int>& getMassIndices() const;
  Vector calculateForce(const Vector& m1State, const Vector& m2State, double massRadii);
};

#endif
