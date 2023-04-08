#include "collision_data.h"
#include "soft_body.h"


/**
 * @brief Collision constructor.
 * 
 * @param time       Time of collision.
 * @param collPoint  Point of collision.
 * @param e          Elasticity of collision.
 */
Collision::Collision(double time, const Vector& collPoint, double e) {
  this->time      = time;
  this->collPoint = collPoint;
  this->e         = e;
}

double Collision::getTime() {
  return time;
}


/**
 * @brief Soft-Rigid body collision constructor.
 * 
 * @param time       Time of collision.
 * @param collPoint  Point of collision.
 * @param surface    Surface of rigid body involved in collision.
 * @param softBody   Soft body involved in collision.
 * @param massIndex  Index of mass in soft body involved in collision.
 * @param e          Elasticity of collision.
 */
SoftRigidCollision::SoftRigidCollision(double time, const Vector& collPoint, const Surface& surface,
                                       SoftBody* softBody, int massIndex, double e)
  : Collision(time, collPoint, e)
{
  this->softBody  = softBody;
  this->surface   = surface;
  this->massIndex = massIndex;
}

void SoftRigidCollision::handle() {
  softBody->handleCollision(time, collPoint, surface, massIndex, e);
}
