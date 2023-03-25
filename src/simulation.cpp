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

void Simulation::addBody(const SoftBody& body) {
  bodies.push_back(body);
}

const std::vector<SoftBody>& Simulation::getBodies() const {
  return bodies;
}

void Simulation::update() {
  double tEnd = time + dt;
  for(SoftBody& body : bodies) {
    body.update(tEnd, iterationsPerUpdate);
  }

  // TODO: Check for collisions


  time = tEnd;
}
