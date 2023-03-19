#include "simulation.h"


Simulation::Simulation(float dt) {
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
  float t_end = time + dt;
  for(SoftBody& body : bodies) {
    body.update(t_end);
  }

  // TODO: Check for collisions


  time = t_end;
}
