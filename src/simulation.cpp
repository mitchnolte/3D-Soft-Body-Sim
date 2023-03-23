#include "simulation.h"


Simulation::Simulation(double dt) {
  this->time = 0;
  this->dt = dt;
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
    body.update(tEnd);
  }

  // TODO: Check for collisions


  time = tEnd;
}
