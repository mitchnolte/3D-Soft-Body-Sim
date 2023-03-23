#ifndef SIMULATION_H
#define SIMULATION_H

#include "soft_body.h"

class Simulation {
  double time;
  double dt;
  std::vector<SoftBody> bodies;

public:
  Simulation(double dt);
  void addBody(const SoftBody& body);
  const std::vector<SoftBody>& getBodies() const;
  void update();
};

#endif
