#include <cstdlib>
#include "simulation.h"
#include "soft_body.h"
#include "soft_cube.h"
#include "rigid_body.h"
#include "collision_data.h"


/**
 * @brief  Simulation constructor.
 *
 * @param  dt                   Time between each update.
 * @param  iterationsPerUpdate  Number of RK4 iterations per update.
 */
Simulation::Simulation(double dt, int iterationsPerUpdate) {
  this->dt = dt;
  this->time = 0;
  this->iterationsPerUpdate = iterationsPerUpdate;
}

Simulation::~Simulation() {
  for(int i=0; i<softBodies.size(); i++)
    delete softBodies[i];
  for(int i=0; i<rigidBodies.size(); i++)
    delete rigidBodies[i];
}

void Simulation::addBody(const SoftCube& body) {
  softBodies.push_back(new SoftCube(body));
}

void Simulation::addBody(const RigidRectPrism& body) {
  rigidBodies.push_back(new RigidRectPrism(body));
}

const std::vector<SoftBody*>& Simulation::getSoftBodies() const {
  return softBodies;
}


/**
 * @brief Advance the simulation forward one time step.
 */
void Simulation::update() {
  double tEnd = time + dt;

  // Calculate soft body states at end of update step
  std::vector<const VecList*> sbStates(softBodies.size());
  for(int i=0; i<softBodies.size(); i++)
    sbStates[i] = &softBodies[i]->calculateUpdatedState(tEnd, iterationsPerUpdate);

  // Collision detection
  std::vector<Collision*> collisions;
  std::unordered_set<int> collidingSoftBodies; // Indices of bodies that require collision handling
  for(int i=0; i<sbStates.size(); i++) {
    for(int j=0; j<rigidBodies.size(); j++) {

      // Bounding sphere test
      if(vecNorm(rigidBodies[j]->getCenterOfMass() - softBodies[i]->getCenterOfMass())
         <= rigidBodies[j]->getBoundingRadius() + softBodies[i]->getBoundingRadius())
      {
        // Full collision test
        rigidBodies[j]->detectCollisions(collisions, softBodies[i], *sbStates[i], time, dt);
        collidingSoftBodies.insert(i);
      }
    }
  }

  // Collision response
  if(collisions.size() > 0) {

    // Sort collisions by time
    std::sort(collisions.begin(), collisions.end(), [](Collision* a, Collision* b) {
      return a->getTime() < b->getTime();
    });

    // Back up time, handle collisions, and advance back to current time
    for(int i : collidingSoftBodies)  softBodies[i]->resetStateBuffer();
    for(Collision* coll : collisions) coll->handle();
    for(int i : collidingSoftBodies)  softBodies[i]->calculateUpdatedState(tEnd, 1);

    for(int i=0; i<collisions.size(); i++)
      delete collisions[i];
  }

  // Update soft bodies
  for(int i=0; i<softBodies.size(); i++)
    softBodies[i]->update();
  time = tEnd;
}
