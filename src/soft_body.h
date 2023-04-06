#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include <GL/glew.h>
#include <unordered_map>
#include "vector.h"
#include "rk4_solver.h"
class RigidRectPrism;

class Mass;
class Spring;


/**
 * @brief Information about a surface used in handling resting contact.
 */
struct Surface {
  const Vector* normal;
  double staticFriction;
  double kineticFriction;
};


/**
 * @brief Interface for a soft body. Implementations define the shape of the
 *        mass-spring structure.
 */
class SoftBody {
protected:
  Vector centerOfMass;              // Approximated center of mass
  std::vector<Mass> masses;
  std::vector<Spring> springs;
  std::vector<int> surfaceMasses;   // Indices of surface masses in mass list
  double mass;                      // Overall body mass
  double massRadii;                 // Radius of a single point mass for internal collision
  double boundingRadius;            // Radius of bounding sphere for collision detection
  double time;                      // Time of state currently stored in masses
  MultiStateRK4solver solver;       // ODE solver

  // Maps an index of a mass to the normal vector of the surface the mass
  // currently has resting contact with.
  std::unordered_map<int, Surface> restCollisions;


  void initSolver(const VecList& state=VecList());
  virtual void approximateCOM(const VecList& state) = 0;

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
  void handleCollision(double tColl, const RigidRectPrism* rigidBody,
                       int massIndex, int faceIndex, double e);

  void updateRestCollisions();
  const VecList& calculateUpdatedState(double time, int RK4iterations);
  void resetStateBuffer();
  void flushStateBuffer();

  virtual const Vector& getCenterOfMass() const = 0;
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
  double restLen;             // Length of spring at rest
  double k;                   // Spring coefficient
  double c;                   // Damping coefficient

public:
  Spring(int mass1, int mass2, double k, double c, double restLen);
  const std::pair<int, int>& getMassIndices() const;
  Vector calculateForce(const Vector& m1State, const Vector& m2State, double massRadii);
};

#endif
