#include "simulation.h"


/**
 * @brief Constructor.
 * @param dt Time between each update.
 * @param iterationsPerUpdate Number of RK4 iterations per update.
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

void Simulation::update() {
  double tEnd = time + dt;

  // Calculate soft body states at end of update step
  std::vector<const VecList*> softBodyStates(softBodies.size());
  for(int i=0; i<softBodies.size(); i++) {
    softBodyStates[i] = &softBodies[i]->calculateUpdatedState(tEnd, iterationsPerUpdate);
  }

  // Check for collisions
  for(SoftBody* softBody : softBodies) {
    for(RigidBody* rigidBody : rigidBodies) {
      if(vecNorm(rigidBody->getCenterOfMass() - softBody->getCenterOfMass())
         <= rigidBody->getBoundingRadius() + softBody->getBoundingRadius())
      {
        // Check for actual collision
      }
    }
  }

  // Update soft bodies
  for(int i=0; i<softBodies.size(); i++) {
    softBodies[i]->update(*softBodyStates[i]);
  }
  time = tEnd;
}
