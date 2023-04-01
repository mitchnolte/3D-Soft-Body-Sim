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

void Simulation::addBody(const SoftBody& body) {
  softBodies.push_back(new SoftBody(body));
}

void Simulation::addBody(const RigidRectPrism& body) {
  rigidBodies.push_back(new RigidRectPrism(body));
}

const std::vector<SoftBody*>& Simulation::getSoftBodies() const {
  return softBodies;
}

void Simulation::update() {
  double tEnd = time + dt;
  for(SoftBody* body : softBodies) {
    body->update(tEnd, iterationsPerUpdate);
  }

  // TODO: Check for collisions


  time = tEnd;
}
