#ifndef SIMULATION_H
#define SIMULATION_H

#include "soft_body.h"

class Simulation {
  float time;
  float dt;
  std::vector<SoftBody> bodies;

public:
  Simulation(float dt);
  void addBody(const SoftBody& body);
  const std::vector<SoftBody>& getBodies() const;
  void update();
};

#endif
