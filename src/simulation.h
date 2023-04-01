#ifndef SIMULATION_H
#define SIMULATION_H

#include "soft_body.h"
#include "rigid_body.h"

class Simulation {
  double time;
  double dt;
  std::vector<SoftBody*> softBodies;
  std::vector<RigidBody*> rigidBodies;
  int iterationsPerUpdate;

public:
  Simulation(double dt, int iterationsPerUpdate=1);
  ~Simulation();
  void addBody(const SoftBody& body);
  void addBody(const RigidRectPrism& body);
  const std::vector<SoftBody*>& getSoftBodies() const;
  void update();
};

#endif
