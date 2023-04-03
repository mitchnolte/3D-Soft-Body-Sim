#ifndef SIMULATION_H
#define SIMULATION_H

#include "vector.h"
#include "collision_data.h"
class SoftBody;
class SoftCube;
class RigidBody;
class RigidRectPrism;


/**
 * @brief Manages and updates the objects in the simulation.
 */
class Simulation {
  double time;
  double dt;
  std::vector<SoftBody*> softBodies;
  std::vector<RigidBody*> rigidBodies;
  int iterationsPerUpdate;

  void handleCollisions(std::vector<Collision*>& collisions, std::vector<const VecList*>& sbStates);

public:
  Simulation(double dt, int iterationsPerUpdate=1);
  ~Simulation();
  void addBody(const SoftCube& body);
  void addBody(const RigidRectPrism& body);
  const std::vector<SoftBody*>& getSoftBodies() const;
  void update();
};

#endif
