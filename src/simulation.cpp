#include "simulation.h"


Simulation::Simulation(float dt) {
  this->time = 0;
  this->dt = dt;
}

void Simulation::addBody(std::shared_ptr<SoftBody> body) {
  bodies.push_back(std::move(body));
}

const BodyList& Simulation::getBodies() {
  return bodies;
}

void Simulation::update() {
  float t_end = time + dt;
  for(std::shared_ptr<SoftBody> body : bodies) {
    body->update(t_end);
  }

  // TODO: Check for collisions


  time = t_end;
}
