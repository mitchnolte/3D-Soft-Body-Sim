#include "collision_data.h"
#include "soft_body.h"


Collision::Collision(double time, const Vector& collPoint, double e) {
  this->time      = time;
  this->collPoint = collPoint;
  this->e         = e;
}

double Collision::getTime() {
  return time;
}


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
