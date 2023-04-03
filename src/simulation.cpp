#include <cstdlib>
#include "simulation.h"
#include "soft_body.h"
#include "soft_cube.h"
#include "rigid_body.h"


/**
 * @brief  Convenience function to append collision data to the main std::vector
 *         in Simulation::update.
 *
 * @param  colls     Main collision list.
 * @param  newColls  New collision data to append.
 * @param  sbIndex   Soft body index.
 * @param  rbIndex   Rigid body index.
 */
void addCollisions(std::vector<Collision*>& colls, const std::vector<Collision*>& newColls,
                   int sbIndex, int rbIndex)
{
  for(int i=0; i<newColls.size(); i++) {
    S_RRP_Coll* collision = (S_RRP_Coll*)newColls[i];
    collision->softBody  = sbIndex;
    collision->rigidBody = rbIndex;
  }
  colls.insert(colls.end(), newColls.begin(), newColls.end());
}


/**
 * @brief  Simulation constructor.
 *
 * @param  dt                   Time between each update.
 * @param  iterationsPerUpdate  Number of RK4 iterations per update.
 */
Simulation::Simulation(double dt, int iterationsPerUpdate) {
  this->time = 0;
  this->dt = dt;
  this->iterationsPerUpdate = iterationsPerUpdate;
}

Simulation::~Simulation() {
  for(int i=0; i<softBodies.size(); i++) {
    delete softBodies[i];
  }
  for(int i=0; i<rigidBodies.size(); i++) {
    delete rigidBodies[i];
  }
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
  for(int i=0; i<softBodies.size(); i++) {
    sbStates[i] = &softBodies[i]->calculateUpdatedState(tEnd, iterationsPerUpdate);
  }

  // Collision detection
  std::vector<Collision*> collisions;
  for(int i=0; i<sbStates.size(); i++) {
    for(int j=0; j<rigidBodies.size(); j++) {

      // Bounding sphere test
      if(vecNorm(rigidBodies[j]->getCenterOfMass() - softBodies[i]->getCenterOfMass())
         <= rigidBodies[j]->getBoundingRadius() + softBodies[i]->getBoundingRadius())
      {
        // Full collision test
        addCollisions(collisions,
               rigidBodies[j]->detectCollisions(softBodies[i], *sbStates[i], time, dt), i, j);
      }
    }
  }

  // Collision response
  if(collisions.size() > 0) {
    
    // Sort collisions by time
    std::sort(collisions.begin(), collisions.end(), [](Collision* a, Collision* b) {
      return a->time < b->time;
    });

    handleCollisions(collisions, sbStates);
    for(int i=0; i<collisions.size(); i++)
      delete collisions[i];
  }

  // Update soft bodies
  for(int i=0; i<softBodies.size(); i++)
    softBodies[i]->update(*sbStates[i]);
  time = tEnd;
}


void Simulation::handleCollisions(std::vector<Collision*>& collisions,
                                  std::vector<const VecList*>& sbStates)
{
  for(int i=0; i<collisions.size(); i++) {
    
  }
}
