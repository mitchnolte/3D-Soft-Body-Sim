#ifndef SIMULATION_H
#define SIMULATION_H

#include <memory>
#include "soft_body.h"

typedef std::vector<std::shared_ptr<SoftBody> > BodyList;


class Simulation {
  float time;
  float dt;
  BodyList bodies;

public:
  Simulation(float dt);
  void addBody(std::shared_ptr<SoftBody> body);
  const BodyList& getBodies();
  void update();
};

#endif
