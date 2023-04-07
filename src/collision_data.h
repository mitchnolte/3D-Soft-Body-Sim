#ifndef COLLISION_DATA_H
#define COLLISION_DATA_H

#include "vector.h"
class SoftBody;


/**
 * @brief Information about a surface used in collision response.
 */
struct Surface {
  const Vector* normal;
  double staticFriction;
  double kineticFriction;
};


/**
 * @brief Stores information about a collision and calls the appropriate
 *        function for handling when Collision::handle is called.
 */
class Collision {
protected:
  Vector collPoint;   // Approximate point of collision  
  double time;        // Time of collision
  double e;           // Elasticity of collision.

public:
  Collision(double time, const Vector& collPoint, double e);
  double getTime();

  virtual void handle() = 0;
};


/**
 * @brief Implementation of Collision for a collision between a point mass of a
 *        soft body and a surface of a rigid body.
 */
class SoftRigidCollision : public Collision {
  SoftBody* softBody;   // Soft body involved in collision
  Surface surface;      // Surface soft body collided with
  int massIndex;        // Index of mass in soft body that has collided

public:
  SoftRigidCollision(double time, const Vector& collPoint, const Surface& surface,
                     SoftBody* softBody, int massIndex, double e);
  void handle();
};

#endif
